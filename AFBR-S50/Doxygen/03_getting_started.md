# Getting Started {#getting_started}

The following section gives an brief overview on of the **AFBR-S50 Core Library and API** and shows how to get started using the evaluation platform and example files. If a port to another microcontroller platform is required, refer to the [Porting Guide](@ref porting_guide).

# AFBR-S50 API {#gs_api}

@note All **AFBR-S50 API** related functions, definitions and constants have a prefix `Argus` which is essentially an alias or working title for the **AFBR-S50 Time-of-Flight Sensor** device.

The *AFBR-S50 Core Library* is provided as a static ANSI-C library file ("lib*.a") and the corresponding API is provided as ANSI-C header files ("*.h"). After setting up the linker to link the library, it is sufficient to include the main header in the "/include" folder, "argus.h":
\code #include "argus.h"\endcode

The API utilizes an abstract handler object that contains all internal states for a single time-of-flight sensor device. In this way, it is possible to use the same API with more than a single device. Note, however, that this feature is not fully implemented in the current version and is planned for a future release! After including the header file, the handler object must be created by calling the #Argus_CreateHandle function to obtain a pointer to the newly allocated object. This is done via the standard library function:
\code void * malloc(size_t size) \endcode
If it is required to use a different function, create and overwrite the weakly linked method and implement your own memory allocation algorithm:
\code void * Argus_Malloc(size_t size) { /* ... */ } \endcode

After creation of the handler object, the *AFBR-S50* module must be initialized with the corresponding handler object:
\code status_t status = Argus_Init(hnd, SPI_SLAVE); \endcode

Note that all peripheral modules must be ready to be used before executing any API function. So make sure to initialize the board and its peripherals before initializing the API via `Argus_Init`. 

After calling `Argus_Init`, the device is ready to run and has been setup with default configuration and calibration data. Use the provided API functions to customize the given default configuration to the needs and requirements of the application. 

Note that `Argus_Init` returns #STATUS_OK on successful initialization. In case of any non-zero return value, refer to the \ref faq section

There are two possibilities to operate the device. First, the simple polling method. The measurements are triggered by the main program via calling the #Argus_TriggerMeasurement function any time a new measurement should be started. A new measurement frame is started and after reading the data from the device, the callback is invoked to inform the host application about the data ready event. In the meantime, the host application can poll the module status or execute other tasks. After finishing the measurement, the #Argus_EvaluateData function must be called to obtain measurement data like range from the raw readout data. It is mandatory to call the evaluation method. Otherwise, the raw data buffer is kept occupied and no new measurements can be triggered anymore. The module contains a double buffer architecture, which allows to start the next measurement and evaluate the current measurement data while the device already executes the next measurement frame. After evaluation, the #argus_results_t data structure is filled with all measurement results that can be processed now be processed by the host application depending on the users needs. An example implementation is shown in the @ref gs_simple_example section. 


Please note that the laser safety module might refuse to restart a measurement at the time the function is called. This is due to timing constraints dictated by the laser safety rules. In this case, the function does return with #STATUS_ARGUS_POWERLIMIT instead. Use the \link #Argus_SetConfigurationFrameTime frame time \endlink and similar parameters to adjust the required pause times via the \link #arguscfg configuration API methods\endlink.

The second way of operating the device is to leverage from an periodic interrupt time that invokes a callback to the API in periodic manner. The timer is implemented in the \link #argus_timer timer\endlink interface. Instead of calling the trigger measurement function periodically from the host application, the measurement restarts itself in an autonomous way. Every time, a new raw measurement data set is ready, the measurement data ready callback is invoked by the API. Similar to the previous method, the #Argus_EvaluateData function must be called before the data can be used. Note that not calling the function will lead to measurements are not restarted before the evaluation method is called and the data buffers is freed. In the same manner, a slow data evaluation or much user code to delay the data evaluation method might decrease the measurement frame rate. An example implementation is shown in the @ref gs_adv_example section.

# Build And Run the Examples using MCUXpresso {#gs_mcuxpresso}

In order to run the provided example projects using the *MCUXpresso IDE*, execute the following steps. Please also refer to the getting started guide by NXP in case of any trouble: https://www.nxp.com/docs/en/user-guide/MCUXSDKGSUG.pdf

\note Depending on where you obtain the code, i.e. from the SDK installer or the GitHub repository, certain steps may differ. The projects that come with the SDK installer (found at "[INSTALL_DIR]\Device\Projects\") are provided as zip-file and will be copied into the MCUXpresso workspace. The projects downloaded or cloned from the GitHub repository are already unpacked an must be imported but not copied into the MCUXpresso workspace (otherwise the project references become invalid).

In order to run provided project, execute the following steps:
	
1. Download and install the *MCUXpresso IDE* (recommended v11.3 or higher): 

	1. Go to https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE
	2. Click on *Download* and register or sign in to download the installer.
	3. Install the IDE.
	4. Go to https://mcuxpresso.nxp.com for more information.
	
2. Download and import the KL46z SDK into the MCUXpresso IDE v11.3

	1. Open MCUXpresso IDE, accept the workspace settings by clicking on "Launch".
	2. Click on "Download and Install SDKs" on the "Welcome Page".
	3. Go to the "Processors" tab and type "MKL46" into the filter field, select the "SDK_2.x_MKL46Z256xxx4" SDK and click "Install".
	4. Accept the licenses and click on "Finish".
	5. After the installation has finished, close the "Welcome" view.
		
	@image html 3_1_install_sdk.jpg "Fig. 3.1: Download and install the SDK files into the MCUXpresso IDE." width=800px
	@image latex 3_1_install_sdk.jpg "Fig. 3.1: Download and install the SDK files into the MCUXpresso IDE."
		
3. Import the project archive files:

	1. Go to MCUXpresso IDE -> Quickstart Panel
	2. Click on "Import projects(s) from file system..."
	3. Select your project for import:

		- If importing a project as zip file as provided by the SDK installer:

			- Click on "Browse..." in the "Project archive (zip)" section
			- Browse to "[INSTALL_DIR]\Device\Projects\" (default is "C:\Program Files (x86)\Broadcom\AFBR-S50 SDK\Device\Projects\"
			- Select the required project archive (e.g. "*AFBR_S50_Example_KL46z.zip*") and click "Open"

		- If importing a project from the local directory as provided by cloning/downloading the GitHub repository:

			- Click on "Browse..." in the "Project directory (unpacked)" section
			- Browse to "[REPOSITORY_ROOT]\Projects\MCUXpresso\" 
			- Select the required project root folder (e.g. "*AFBR_S50_Example_KL46z*") and click "Select Folder"
			- **IMPORTANT**: Click "Next" and disable the check at "Copy projects into Workspace". (Otherwise, the references to the source files will be messed up!)

			@image html 3_2_disable_copy_into_ws.jpg "Fig. 3.3: Disable the check at \"Copy projects into workspace\" before importing an directory project!" width=400px
			@image latex 3_2_disable_copy_into_ws.jpg "Fig. 3.3: Disable the check at \"Copy projects into workspace\" before importing an directory project!" 

	4. Click "Finish"
	
	@image html 3_3_import_project.jpg "Fig. 3.3: Import the project archive into the MCUXpresso IDE." width=800px
	@image latex 3_3_import_project.jpg "Fig. 3.3: Import the project archive into the MCUXpresso IDE."

4. Build the projects:

	- Go to MCUXpresso IDE -> Project Explorer and select the project to build.
	- Go to Quickstart Panel and click on "Build"
	
	@image html 3_4_build.jpg "Fig. 3.4: Build the project." width=800px
	@image latex 3_4_build.jpg "Fig. 3.4: Build the project."
	
5. Download and install the OpenSDA drivers:

	- Go to http://www.pemicro.com/opensda/
	- Download and install *PEDrivers_install.exe*
	
6. Debug and run the project with the OpenSDA debugger:

	- Connect the *OpenSDA* USB port of the KL46z evaluation board.
	- Go to MCUXpresso IDE -> Quickstart Panel
	- Click in the PEMicro Icon (see screenshot).
	- The Debug Probe will be discovered and an according window will show.
	- Click on "OK"	
		
	@image html 3_5_debug.jpg "Fig. 3.5: Run and debug the project." width=800px
	@image latex 3_5_debug.jpg "Fig. 3.5: Run and debug the project." 
	
	- If the program breaks at the main() function, hit the "Resume" button.
	
	@image html 3_6_debug_resume.jpg "Fig. 3.6: Resume the halted program." width=800px
	@image latex 3_6_debug_resume.jpg "Fig. 3.6: Resume the halted program." 
	
7. Display the measurement values on a PC via an UART terminal:

	- Open a terminal (e.g. [Termite](https://www.compuphase.com/software_termite.htm)) and open a UART connection using 115200 bps, 8N1, no handshake connection. 
	
	@image html 3_7_terminal_settings.jpg "Fig. 3.7: Setting up the terminal to receive measurement results." width=600px
	@image latex 3_7_terminal_settings.jpg "Fig. 3.7: Setting up the terminal to receive measurement results." width=0.6\textwidth
		
	- Range values will start to occur on the terminal as soon as the program starts its execution.

	@image html 3_8_terminal.jpg "Fig. 3.8: The serial terminal is used to display the received measurement results." width=600px
	@image latex 3_8_terminal.jpg "Fig. 3.8: The serial terminal is used to display the received measurement results." width=0.6\textwidth
	
	
The example application is now up and running. It is a simple program that executes measurements and displays the 1D distance (Units: millimeter) value on the terminal. See also the [AFBR-S50 API](#gs_api) section for a brief description.
	
The evaluation kit is build on the FRDM-KL46z Evaluation Kit from NXP. So you may also refer https://www.nxp.com/frdm-kl46z for further information.


# Simple Example {#gs_simple_example}

Here is an example of how to use the API in a simple loop with polling the module status to wait for the measurement data to be ready. The 1D range value of the obtained measurement data is streamed via an UART connection. Open a terminal (e.g. [Termite](https://www.compuphase.com/software_termite.htm)) and open a UART connection using 115200 bps, 8N1, no handshake connection. Range values will start to occur on the terminal as soon as the program starts its execution.

Please find the example files in "[INSTALL_DIR]\Device\Examples\" (default is "C:\Program Files (x86)\Broadcom\AFBR-S50 SDK\Device\Examples\").

\include 01_simple_example.c

# Advanced Example {#gs_adv_example}

Here is an example of how to use the API with the autonomous measurement triggering. The 1D range value of the obtained measurement data is streamed via an UART connection. Open a terminal (e.g. [Termite](https://www.compuphase.com/software_termite.htm)) and open a UART connection using 115200 bps, 8N1, no handshake connection. Range values will start to occur on the terminal as soon as the program starts its execution.

Please find the example files in "[INSTALL_DIR]\Device\Examples\" (default is "C:\Program Files (x86)\Broadcom\AFBR-S50 SDK\Device\Examples\").

\include 02_advanced_example.c


\example 01_simple_example.c
\example 02_advanced_example.c