#pragma once
/*****************
NetworkedModule.h
ALEXANDER CUNIO 2024
*****************/

/*
This class provides additional functionality for networked modules (Laser, GPS, Vehicle control, Display) beyond the
base UGV module class. These modules should therefore be derived from this rather than the base class.
For clarification of inheritance requirements see diagram in the assignment specification.
*/

#include <UGVModule.h>

// You will need to select which address to use depending on if you are working with the simulator (127.0.0.1)
// or the physical robot in the lab (192.168.1.200).
#define WEEDER_ADDRESS "127.0.0.1"
//#define WEEDER_ADDRESS "192.168.1.200"
#define DISPLAY_ADDRESS "127.0.0.1" // Display is always running locally (run the MATLAB code separately)

namespace ports {
	static const int LiDAR = 23000;
	static const int GNSS = 24000;
	static const int VC = 25000;
	static const int DISPLAY = 28000;
}

ref class NetworkedModule abstract : public UGVModule
{
public:
	// Establish TCP connection
	virtual error_state connect(String^ hostName, int portNumber) {
		disconnect();
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
			return error_state::SUCCESS;
		}

		catch (Exception^)
		{
			disconnect();
			return error_state::ERR_STARTUP;
		}
	}

	virtual error_state communicate() = 0;								// Communicate over TCP connection (includes any error checking required on data)
	void disconnect()													// Release existing connections
	{
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
	}
	virtual error_state authenticate(String^ id) { return error_state::ERR_CONNECTION;  }
	
	protected:
		TcpClient^ Client;					// Handle for TCP connection
		NetworkStream^ Stream;				// Handle for TCP data stream
		array<unsigned char>^ ReadData;		// Array to store sensor Data (only used for sensor modules)
};
