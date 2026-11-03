#include "LiDAR.h"

using namespace System::IO;


LiDAR::LiDAR(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_LiDAR)
{
	SM_LiDAR_ = SM_LiDAR;
	SM_TM_ = SM_TM;
	LiDAR_Stopwatch = gcnew Stopwatch;
}

error_state LiDAR::connect(String^ hostName, int portNumber)
{
	Console::WriteLine("Attempting to connect to LiDAR");
	disconnect();

	ReadData = gcnew array<unsigned char>(2048);

	try
	{
		Client = gcnew TcpClient();

		Client->ReceiveBufferSize = 2048;
		Client->ReceiveTimeout = 500;
		Client->NoDelay = true;

		IAsyncResult^ asyncResult = Client->BeginConnect(hostName, portNumber, nullptr, nullptr);
		bool connected = asyncResult->AsyncWaitHandle->WaitOne(1500, false);

		if (!connected)
		{
			disconnect();
			return error_state::ERR_CONNECTION;
		}

		Client->EndConnect(asyncResult);


		if (!Client->Connected)
		{
			disconnect();
			return error_state::ERR_STARTUP;
		}

		Stream = Client->GetStream();
		ReadData = gcnew array<unsigned char>(Client->ReceiveBufferSize);

		Console::WriteLine("Connection Success");
		return error_state::SUCCESS;
	}

	catch (Exception^)
	{
		disconnect();
		return error_state::ERR_STARTUP;
	}
}

error_state LiDAR::authenticate(String^ id) {
	Console::WriteLine("Attempting to authenticate with LiDAR Service");
	String^ command = id + "\n";
	//sendCommand(command);

	if (ReadData == nullptr || ReadData->Length == 0)
	{
		ReadData = gcnew array<unsigned char>(2048);
	}

	array<unsigned char>^ SendData = Encoding::ASCII->GetBytes(command);
	Stream->Write(SendData, 0, SendData->Length);

	Thread::Sleep(500);

	int bytesRead = Stream->Read(ReadData, 0, ReadData->Length);
	String^ Response = Encoding::ASCII->GetString(ReadData, 0, bytesRead);

	if (Response == "OK\n") {
		Console::WriteLine("LiDAR Authenticated");
		return error_state::SUCCESS;
	}
	return error_state::ERR_CONNECTION;
}

error_state LiDAR::communicate()
{
	if (ReadData == nullptr || ReadData->Length < 8192)
	{
		ReadData = gcnew array<unsigned char>(8192);
	}

	sendCommand("sRN LMDscandata");

	int totalBytes = 0;
	int bytesRead = 0;
	bool started = false;

	while (totalBytes < ReadData->Length)
	{
		int b = Stream->ReadByte();
		if (b == -1)
		{
			return error_state::ERR_CONNECTION;
		}
		else if (b == 0x02)
		{
			totalBytes = 0;
			started = true;
		}

		else if (b == 0x03 && started)
		{
			break;
		}
		else if (started)
		{
			ReadData[totalBytes++] = (unsigned char)b;
		}
	}
	
	String^ response = Encoding::ASCII->GetString(ReadData, 0, totalBytes);
	// Trim 0x02 and 0x03
	array<String^>^ tokens = response->Split(' ');


	int distIndex = -1;

	for (int i = 0; i < tokens->Length; i++)
	{
		if (tokens[i]->Trim() == "DIST1")
		{
			distIndex = i;
			break;
		}
	}

	if (distIndex == -1)
	{
		Console::WriteLine("Dist1 Not Found");
		return error_state::ERR_INVALID_DATA;
	}

	int numPoints = Convert::ToInt32(tokens[distIndex + 5], 16);
	int dataStartIndex = distIndex + 6;

	if (numPoints != STANDARD_LASER_LENGTH || (dataStartIndex + numPoints) > tokens->Length)
	{
		return error_state::ERR_INVALID_DATA;
	}

	// Store data
	for (int i = 0; i < STANDARD_LASER_LENGTH; i++)
	{
		uint16_t distMm = Convert::ToUInt16(tokens[dataStartIndex + i], 16);
		double angleRad = (i * 0.5) * (Math::PI / 180.0);

		Monitor::Enter(SM_LiDAR_);
		try
		{
			double x = -distMm * Math::Sin(angleRad);
			double y = -distMm * Math::Cos(angleRad);
			SM_LiDAR_->x[i] = x;
			SM_LiDAR_->y[i] = y;
			// Console::Write("{0}, {1}", x, y);
		}
		catch (Exception^)
		{
			Console::WriteLine("Writing to SM_LiDAR_ error");
		}
		finally
		{
			Monitor::Exit(SM_LiDAR_);
		}

	}

	return error_state::SUCCESS;
}

void LiDAR::sendCommand(String^ command) {
	String^ cleanCommand = command->Trim(
		gcnew array<wchar_t>{' ', '\n', '\r', '\0', (wchar_t)0x02, (wchar_t)0x03}
	);


	array<unsigned char>^ cmdBytes = Encoding::ASCII->GetBytes(cleanCommand);
	array<unsigned char>^ packet = gcnew array<Byte>(cmdBytes->Length + 2);

	packet[0] = 0x02;
	Array::Copy(cmdBytes, 0, packet, 1, cmdBytes->Length);
	packet[packet->Length - 1] = 0x03;

	Stream->Write(packet, 0, packet->Length);
}

void LiDAR::outputDataToTerminal()
{
	static int frameCount = 0;
	frameCount++;

	if (frameCount >= 100)
	{
		frameCount = 0;

		StringBuilder^ sb = gcnew StringBuilder();

		Monitor::Enter(SM_LiDAR_);
		try {
			int pairsPerLine = 4;
			for (int i = 0; i < 361; i++)
			{
				// Format each point as: [Idx] (   X,    Y)
				sb->AppendFormat("[{0,3}] ({1,6:F0},{2,6:F0})  ", i, SM_LiDAR_->x[i], SM_LiDAR_->y[i]);

				// Move to the next line after 4 pairs, or at the very end of the 361 points
				if ((i + 1) % pairsPerLine == 0 || i == 360)
				{
					sb->AppendLine();
				}
			}
			sb->AppendLine("========================================================================");

			Console::Write(sb->ToString());
		}
		catch (Exception^)
		{
			Console::WriteLine("could not access SM_LIDAR");
		}
		finally
		{
			Monitor::Exit(SM_LiDAR_);
		}
	}
}

error_state LiDAR::SetupSharedMemory()
{
	return error_state::SUCCESS;
}

error_state LiDAR::processHeartBeats()
{
	// Check LiDAR bit in heartbeat is low
	if ((SM_TM_->heartbeat & bit_LASER) == 0)
	{
		SM_TM_->heartbeat |= bit_LASER;
		LiDAR_Stopwatch->Restart();
	}

	return error_state::SUCCESS;
}

error_state LiDAR::processSharedMemory()
{
	return error_state::SUCCESS;
}

bool LiDAR::getShutdownFlag()
{
	return SM_TM_->shutdown & bit_LASER;
}

void LiDAR::selfShutDown()
{
	SM_TM_->shutdown |= bit_LASER;
}

void LiDAR::threadFunction()
{
	Console::WriteLine("LiDAR Thread Starting");

	error_state result = connect(WEEDER_ADDRESS, ports::LiDAR);
	if (result != error_state::SUCCESS)
	{
		Console::WriteLine("LiDAR Connection Error");
		printError(result);
		selfShutDown();
	}

	else if (authenticate("5480558") != error_state::SUCCESS)
	{
		Console::WriteLine("LiDAR Authentication Failure");
		selfShutDown();
	}
	SM_TM_->ThreadBarrier->SignalAndWait();

	LiDAR_Stopwatch->Start();
	Console::WriteLine(getShutdownFlag());
	while (!getShutdownFlag())
	{
		error_state response;
		try
		{
			response = communicate();

		}

		catch (Exception^)
		{
			selfShutDown();
		}
		// Console::WriteLine("LiDAR Thread is running");
		if (response == error_state::SUCCESS)
		{
			processSharedMemory();
			processHeartBeats();
			// Console::WriteLine("lidar alive");
			outputDataToTerminal();

		}
		Thread::Sleep(10);
	}
	Console::WriteLine("LiDAR Thread terminated");

	return;
}

