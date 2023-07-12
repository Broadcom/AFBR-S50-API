# MCUXpresso IDE {#mcuxpresso}

The provided example projects can be run on the **NXP** targets (**MKL46Z**,
**MKL17Z**) using the **MCUXpresso IDE**. The evaluation kit is build on the
**FRDM-KL46z Evaluation Kit from NXP**. Also refer to
<https://www.nxp.com/frdm-kl46z> for further information.

@note This guide shows how to use the example projects but the same steps can be
executed to run the other **MCUXpresso IDE** projects. See \ref apps for an
overview.

Execute the following steps to get the example projects up and running. Please
also refer to the
[getting started guide by NXP](https://www.nxp.com/docs/en/user-guide/MCUXSDKGSUG.pdf)
in case of any trouble.

@note Depending on where you obtain the code, i.e. from the SDK installer or the
GitHub repository, certain steps may differ. The projects that come with the SDK
installer (found at `[INSTALL_DIR]\Device\Projects\`) are provided as zip-file
and will be copied into the MCUXpressoIDE workspace. The projects downloaded or
cloned from the
[AFBR-S50 GitHub repository](https://github.com/Broadcom/AFBR-S50-API) are
already unpacked an must be imported but not copied into the MCUXpressoIDE
workspace (otherwise the project references may become invalid).

In order to run provided project, execute the following steps:

## Install MCUXpresso IDE

Download and install the **MCUXpresso IDE** (recommended v11.5 or higher):

1. Go to
   <https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE>
2. Click on `Download` and register or sign in to download the installer.
3. Install the IDE.
4. Go to <https://mcuxpresso.nxp.com> for more information.

## Import NXP MKL46z SDK

Download and import the **NXP MKL46z SDK** (or **NXP MKL17z SDK**) into the
**MCUXpresso IDE**

1. Open MCUXpresso IDE, accept the workspace settings by clicking on `Launch`.
2. Click on `Download and Install SDKs` on the `Welcome Page`.
3. Go to the `Processors` tab and type `MKL46` into the filter field, select the
   `SDK_2.x_MKL46Z256xxx4` (or `SDK_2.x_MKL17Z256xxx4`) SDK and click `Install`
4. Accept the licenses and click on `Finish`.
5. After the installation has finished, close the `Welcome` view.

@image html 3_1_install_sdk.jpg "Fig. 3.1: Download and install the SDK files into the MCUXpresso IDE." width=800px
@image latex 3_1_install_sdk.jpg "Fig. 3.1: Download and install the SDK files into the MCUXpresso IDE."

## Import a Project

Import the Project archive files:

1. Go to **MCUXpresso IDE** -> `Quickstart Panel`
2. Click on `Import projects(s) from file system...`
3. Select your project for import:

    -   If importing a project as zip file as provided by the **SDK installer**:

        1. Click on `Browse...` in the `Project archive (zip)` section
        2. Browse to `[INSTALL_DIR]\Device\Projects\` (default is
           `C:\Program Files (x86)\Broadcom\AFBR-S50 SDK\Device\Projects\`)
        3. Select the required project archive (e.g.
           `AFBR_S50_Example_KL46z.zip`) and click `Open`
            @image html 3_2_import_project.jpg "Fig. 3.2: Import the project archive into the MCUXpresso IDE." width=800px
            @image latex 3_2_import_project.jpg "Fig. 3.2: Import the project archive into the MCUXpresso IDE."

    -   If importing a project from the local directory as provided by
        cloning/downloading the **GitHub repository**:

        1. Click on `Browse...` in the `Project directory (unpacked)` section
        2. Browse to `[REPOSITORY_ROOT]\Projects\MCUXpressoIDE\`
        3. Select the required project root folder (e.g.
           `AFBR_S50_Example_KL46z`) and click `Select Folder`
        4. **IMPORTANT**: Click `Next`and disable the check at
           `Copy projects into Workspace`. (Otherwise, the references to the
           source files will be messed up!)
            @image html 3_3_disable_copy_into_ws.jpg "Fig. 3.3: Disable the check at \"Copy projects into workspace\" before importing an directory project!" width=400px
            @image latex 3_3_disable_copy_into_ws.jpg "Fig. 3.3: Disable the check at \"Copy projects into workspace\" before importing an directory project!"

4. Click `Finish`

## Build the Project

\note Usually, the first slave (`SPI_SLAVE = 1`) identifier is used for all
current evaluation or reference boards. Only some deprecated evaluation boards
based on the FRDM-KL46z evaluation board may use the fifth (`SPI_SLAVE = 5`) SPI
slave which needs to be set manually in the `example.h` (within
`Project > App > examples > example.h`) file by changing the `SPI_SLAVE`
preprocessor definition accordingly.

1. Go to **MCUXpresso IDE** -> `Project Explorer` and select the project to
   build.
2. Go to `Quickstart Panel` and click on `Build`

@image html 3_4_build.jpg "Fig. 3.4: Build the project." width=800px
@image latex 3_4_build.jpg "Fig. 3.4: Build the project."

## Download and install the OpenSDA drivers

1. Go to <http://www.pemicro.com/opensda/>
2. Download and install `PEDrivers_install.exe`

## Debug the Project with the OpenSDA Debugger

1. Connect the `OpenSDA` USB port of the FRMD-KL46z evaluation board.
2. Go to **MCUXpresso IDE** -> `Quickstart Panel`
3. Click in the PEMicro Icon (see screenshot).
4. The Debug Probe will be discovered and an according window will show.
5. Click on `OK`
    @image html 3_5_debug.jpg "Fig. 3.5: Run and debug the project." width=800px
    @image latex 3_5_debug.jpg "Fig. 3.5: Run and debug the project."
6. If the program breaks at the `main()` function, hit the `Resume` button.
    @image html 3_6_debug_resume.jpg "Fig. 3.6: Resume the halted program." width=800px
    @image latex 3_6_debug_resume.jpg "Fig. 3.6: Resume the halted program."

## Display the Measurement Values

@note This applies only for the \ref example_app. In case of the \ref
explorer_app, use the AFBR-S50 Explorer GUI instead.

Display the measurement values on a PC via an UART terminal:

1. Open a terminal (e.g.
   [Termite](https://www.compuphase.com/software_termite.htm)) and open a
   UART connection using 115200 bps, 8N1, no handshake connection.
   | Baud rate  | Data Bits | Stop Bits | Parity | Flow Control |
   | ---------- | --------- | --------- | ------ | ------------ |
   | 115200 bps | 8         | 1         | none   | none         |
   @image html 3_7_terminal_settings.jpg "Fig. 3.7: Setting up the terminal to receive measurement results." width=600px
   @image latex 3_7_terminal_settings.jpg "Fig. 3.7: Setting up the terminal to receive measurement results." width=0.6\textwidth

2. Range values will start to occur on the terminal as soon as the program
   starts its execution.
   @image html 3_8_terminal.gif "Fig. 3.8: The serial terminal is used to display the received measurement results." width=600px
   @image latex 3_8_terminal.jpg "Fig. 3.8: The serial terminal is used to display the received measurement results." width=0.6\textwidth
