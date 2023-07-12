# STM32CubeIDE {#stm32cubeide}

The provided example projects can be run on the **STM32** targets
(**STM32F401RE**) using the **STM32Cube IDE**.

@note This guide shows how to use the example projects but the same steps can be
executed to run the other **STM32Cube IDE** projects. See \ref apps for an
overview.

Execute the following steps to get the example projects up and running. Please
also refer to the
[STM32CubeIDE documentation by STM](https://www.st.com/en/development-tools/stm32cubeide.html)
in case of any trouble.

@note Depending on where you obtain the code, i.e. from the SDK installer or the
GitHub repository, certain steps may differ. The projects that come with the SDK
installer (found at `[INSTALL_DIR]\Device\Projects\`) are provided as zip-file
and will be copied into the STM32CubeIDE workspace. The projects downloaded or
cloned from the
[AFBR-S50 GitHub repository](https://github.com/Broadcom/AFBR-S50-API) are
already unpacked an must be imported but not copied into the STM32CubeIDE
workspace (otherwise the project references may become invalid).

In order to run provided project, execute the following steps:

## Install STM32CubeIDE

Download and install the **STM32CubeIDE**:

1. Go to <https://www.st.com/en/development-tools/stm32cubeide.html>
2. Click on `Get Software` and select and download the correct installer.
3. Install the IDE.
4. Go to <https://www.st.com/en/development-tools/stm32cubeide.html> for more
   information.

## Import a Project

Import the project archive files:

1. Go to **STM32CubeIDE** and close the `Welcome Page`
2. Click on `Menu` > `File` > `Import...`
3. Select `General` > `Existing Projects into Workspace` and click `Next`
4. Select your project for import:

    - If importing a project as zip file as provided by the **SDK installer**:

        1. Click on `Browse...` in the `Select archive file` section
        2. Browse to `[INSTALL_DIR]\Device\Projects\` (default is
           `C:\Program Files (x86)\Broadcom\AFBR-S50 SDK\Device\Projects\`
        3. Select the required project archive (e.g.
           `AFBR_S50_Example_STM32F04RE.zip`) and click `Open`

    - If importing a project from the local directory as provided by
      cloning/downloading the **GitHub repository**:

        1. Click on `Browse...` in the `Select root directory` section
        2. Browse to `[REPOSITORY_ROOT]\Projects\STM32CubeIDE\`
        3. Select the required project root folder (e.g.
           `AFBR_S50_Example_F401RE`) and click `Select Folder`
        4. **IMPORTANT**: Click `Next`and disable the check at
           `Copy projects into Workspace`. (Otherwise, the references to the
           source files will be messed up!)
           @image html 3_13_import_project.PNG "Fig. 3.13: Import Project to STM32CubeIDE" width=800px
           @image latex 3_13_import_project.PNG "Fig. 3.13: Import Project to STM32CubeIDE"

5. Click `Finish`

## Build the Project

1. Go to **STM32CubeIDE** -> `Project Explorer` and select the project to build.
2. Go to `Menu` -> `Project` -> `Build Project` to build the project.

@image html 3_14_build_project.PNG "Fig. 3.14: Build the project on STM32CubeIDE." width=800px
@image latex 3_14_build_project.PNG "Fig. 3.14: Build the project on STM32CubeIDE."

## Debug the Project with the STLink debugger

1. Connect the STM32F401RE Nucleo Board via USB cable.
2. Select the project to debug in the `Project Explorer` of the `STM32CubeIDE`.
3. Select `Run` > `Debug As` > `STM32 C/C++ Application`.
4. Click on `OK` (Everything should be setup correctly by default).
    @image html 3_15_debug_config.PNG "Fig. 3.15: Run and debug the project." width=800px
    @image latex 3_15_debug_config.PNG "Fig. 3.15: Run and debug the project."
5. If the program breaks at the `main()` function, hit the `Resume` button.

## Connect the UART interface to PC

Follow the steps in the @ref reference_board_uart section to connect to the
board via UART.

## Display the Measurement Values

@note This applies only for the \ref example_app. In case of the \ref explorer_app,
use the AFBR-S50 Explorer GUI instead.

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
