#include "GNSS.h"

#define CRC32_POLYNOMIAL 0xEDB88320L

using namespace System;
using namespace System::Collections::Generic;
using namespace System::Threading;

// Helper functions. Reads Little-Endian Primitives 
static unsigned long CRC32Value(int i)
{
	int j;
	unsigned long ulCRC = i;
	for (j = 8; j > 0; j--)
	{
		if (ulCRC & 1)
			ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
		else
			ulCRC >>= 1;
	}
	return ulCRC;
}

static unsigned long CalculateBlockCRC32(unsigned long ulCount, unsigned char* ucBuffer)
{
	unsigned long ulTemp1;
	unsigned long ulTemp2;
	unsigned long ulCRC = 0;

	while (ulCount-- != 0)
	{
		ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
		ulTemp2 = CRC32Value(((int)ulCRC ^ *ucBuffer++) & 0xff);
		ulCRC = ulTemp1 ^ ulTemp2;
	}
	return ulCRC;
}

static unsigned short ReadUInt16(List<unsigned char>^ buf, int offset) {
	array<unsigned char>^ tmp = gcnew array<unsigned char>(2);

	tmp[0] = buf[offset];
	tmp[1] = buf[offset + 1];

	return BitConverter::ToUInt16(tmp, 0);
}

static unsigned int ReadUInt32(List<unsigned char>^ buf, int offset) {
	array<unsigned char>^ tmp = gcnew array<unsigned char>(4);

	for (int i = 0; i < 4; i++) {
		tmp[i] = buf[offset + i];
	}
	return BitConverter::ToUInt32(tmp, 0);
}

static double ReadDouble(List<unsigned char>^ buf, int offset) {
	array<unsigned char>^ tmp = gcnew array<unsigned char>(8);

	for (int i = 0; i < 8; i++) {
		tmp[i] = buf[offset + i];
	}

	return BitConverter::ToDouble(tmp, 0);
}



GNSS::GNSS(SM_ThreadManagement^ SM_TM, SM_GPS^ SM_GNSS) {
	SM_TM_ = SM_TM;
	SM_GNSS_ = SM_GNSS;
	
	GNSS_Stopwatch = gcnew Stopwatch;
	rxBuffer_ = gcnew List<unsigned char>();
	ReadData = gcnew array<unsigned char>(1024);
}

void GNSS::disconnect() {
	if (Stream != nullptr)
	{
		try
		{
			Stream->Close();
		}
		catch (Exception^) {}
		finally
		{
			Stream = nullptr;
		}
	}

	if (Client != nullptr)
	{
		try
		{
			Client->Close();
		}
		catch (Exception^) {}
		finally
		{
			Client = nullptr;
		}
	}
	ReadData = nullptr;

	if (rxBuffer_ != nullptr) {
		rxBuffer_->Clear();
	}
}

error_state GNSS::communicate() {
	if (verifyConnection() != error_state::SUCCESS) {
		disconnect();
		return error_state::ERR_CONNECTION;
	}

	if (rxBuffer_ == nullptr) {
		rxBuffer_ = gcnew List<unsigned char>();
	}


	try {
		if (!Stream->DataAvailable) {
			return error_state::ERR_NO_DATA;
		}

		int bytesRead = Stream->Read(ReadData, 0, ReadData->Length);

		
		if (bytesRead <= 0) {
			return error_state::ERR_NO_DATA;
		}

		for (int i = 0; i < bytesRead; i++) {
			rxBuffer_->Add(ReadData[i]);
		}

		const unsigned char SYNC_1 = 0xAA; // 170
		const unsigned char SYNC_2 = 0x44; // 68
		const unsigned char SYNC_3 = 0x12; // 18
		const unsigned short BESTUTM_MSG_ID = 726; 
		const int CRC_SIZE = 4;

		bool parsedValidPacket = false;
		bool crcMismatch = false;


		while (rxBuffer_->Count >= 3) {
			// Align to 3 byte sync sequence
			if (rxBuffer_[0] != SYNC_1 || rxBuffer_[1] != SYNC_2 || rxBuffer_[2] != SYNC_3) {
				rxBuffer_->RemoveAt(0);
				continue;
			}

			// Need at least 10 bytes to read Header Length and Message Length
			if (rxBuffer_->Count < 10) {
				break;
			}

			int headerLen = rxBuffer_[3];
			array<unsigned char>^ lenBytes = gcnew array<unsigned char>(2);

			lenBytes[0] = rxBuffer_[8];
			lenBytes[1] = rxBuffer_[9];
			
			int bodyLen = System::BitConverter::ToUInt16(lenBytes, 0);

			if (headerLen < 10 || bodyLen < 0) {
				rxBuffer_->RemoveAt(0);
				continue;
			}

			int totalPacketLen = headerLen + bodyLen + CRC_SIZE;

			// Wait for full packet length
			if (rxBuffer_->Count < totalPacketLen) {
				break;
			}

			array<unsigned char>^ idBytes = gcnew array<unsigned char>(2);
			idBytes[0] = rxBuffer_[4];
			idBytes[1] = rxBuffer_[5];
			unsigned short msgId = BitConverter::ToUInt16(idBytes, 0);

			// Check for BESTUTM with a body large enough for hgt
			if (msgId == BESTUTM_MSG_ID && bodyLen >= 40) {
				int blockLen = headerLen + bodyLen;
				array<unsigned char>^ block = gcnew array<unsigned char>(blockLen);
				for (int i = 0; i < blockLen; i++) {
					block[i] = rxBuffer_[i];
				}

				pin_ptr<unsigned char> pinnedBlock = &block[0];
				unsigned long calculatedCrc = CalculateBlockCRC32(
					static_cast<unsigned long>(blockLen),
					pinnedBlock
				);

				pinnedBlock = nullptr;

				array<unsigned char>^ crcBytes = gcnew array<unsigned char>(4);
				for (int i = 0; i < 4; i++) {
					crcBytes[i] = rxBuffer_[blockLen + i];
				}

				unsigned long receivedCrc = static_cast<unsigned long>(BitConverter::ToUInt32(crcBytes, 0));

				if (calculatedCrc == receivedCrc) {
					array<unsigned char>^ nBytes = gcnew array<unsigned char>(8);
					array<unsigned char>^ eBytes = gcnew array<unsigned char>(8);
					array<unsigned char>^ hBytes = gcnew array<unsigned char>(8);

					for (int i = 0; i < 8; i++) {
						nBytes[i] = rxBuffer_[headerLen + 16 + i];
						eBytes[i] = rxBuffer_[headerLen + 24 + i];
						hBytes[i] = rxBuffer_[headerLen + 32 + i];
					}

					double northing = BitConverter::ToDouble(nBytes, 0);
					double easting = BitConverter::ToDouble(eBytes, 0);
					double height = BitConverter::ToDouble(hBytes, 0);

					bool lockTaken = false;
					try {
						Monitor::Enter(SM_GNSS_->lockObject, lockTaken);
						SM_GNSS_->Northing = northing;
						SM_GNSS_->Easting = easting;
						SM_GNSS_->Height = height;
					}
					finally {
						if (lockTaken) {
							Monitor::Exit(SM_GNSS_->lockObject);
						}
					}

					System::Console::WriteLine(
						"Northing: {0:F3} m, Easting: {1:F3} m, Height: {2:F3} m | Rx CRC: 0x{3:X8}, Calc CRC: 0x{4:X8}",
						northing, easting, height, receivedCrc, calculatedCrc
					);
					parsedValidPacket = true;
				}
				else {
					System::Console::WriteLine(
						"GNSS CRC mismatch | Rx CRC: 0x{0:X8}, Calc CRC: 0x{1:X8}",
						receivedCrc, calculatedCrc
					);
					crcMismatch = true;
				}
			}
			rxBuffer_->RemoveRange(0, totalPacketLen);

		}

		if (parsedValidPacket) {
			return error_state::SUCCESS;
		}
		if (crcMismatch) {
			return error_state::ERR_INVALID_DATA;
		}
		return error_state::ERR_NO_DATA;

	}

	catch (System::IO::IOException^) {
		Console::WriteLine("io exception");
		disconnect();
		return error_state::ERR_CONNECTION;
	}

	catch (System::Net::Sockets::SocketException^) {
		Console::WriteLine("socket exception");

		disconnect();
		return error_state::ERR_CONNECTION;
	}
}

error_state	GNSS::SetupSharedMemory() {
	//bool lockTaken;
	//try {
	//	Monitor::Enter(SM_GNSS_->lockObject, lockTaken);
	//	SM_GNSS_->Northing = -1;
	//	SM_GNSS_->Easting = -1;
	//	SM_GNSS_->Height = -1;
	//}
	//finally {
	//	if (lockTaken) {
	//		Monitor::Exit(SM_GNSS_->lockObject);
	//	}
	//}
	return error_state::SUCCESS;
}

error_state GNSS::processHeartBeats() {
	if ((SM_TM_->heartbeat & bit_GPS) == 0) {
		SM_TM_->heartbeat |= bit_GPS;
		GNSS_Stopwatch->Restart();
	}
	return error_state::SUCCESS;
}

error_state GNSS::processSharedMemory() {
	return error_state::SUCCESS;
}

bool GNSS::getShutdownFlag() {
	return SM_TM_->shutdown & bit_GPS;
}

void GNSS::selfShutDown() {
	SM_TM_->shutdown |= bit_GPS;
}

void GNSS::threadFunction() {
	Console::WriteLine("GNSS Thread Starting");

	error_state result = connect(WEEDER_ADDRESS, ports::GNSS);
	if (result != error_state::SUCCESS) {
		Console::WriteLine("GNSS Connection Error");\
		selfShutDown();
		SM_TM_->ThreadBarrier->SignalAndWait();
		Console::WriteLine("GNSS Thread Terminated");
		return;
	}

	SM_TM_->ThreadBarrier->SignalAndWait();

	GNSS_Stopwatch->Start();
	while (!getShutdownFlag()) {
		error_state response = error_state::ERR_NO_DATA;
		try {
			response = communicate();
		}

		catch (Exception^) {
			selfShutDown();
			break;
		}

		if (response == error_state::SUCCESS) {
			processHeartBeats();
		}
		else if (response == error_state::ERR_CONNECTION) {
			selfShutDown();
		}
		Thread::Sleep(10);
	}

	disconnect();
	Console::WriteLine("GNSS Thread Terminated");
}
