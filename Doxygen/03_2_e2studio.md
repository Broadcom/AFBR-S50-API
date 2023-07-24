# e² Studio IDE {#e2studio}

The provided example projects can be run on the **Renesas** targets (**RA4M2**)
using the **e² Studio IDE**.

@note This guide shows how to use the example projects but the same steps can be
executed to run the other **e² Studio IDE** projects. See \ref apps for an
overview.

Execute the following steps to get the example projects up and running. Please
also refer to the
[e² Studio documentation by Renesas](https://www.renesas.com/us/en/software-tool/e-studio)
in case of any trouble.

@note Depending on where you obtain the code, i.e. from the SDK installer or the
GitHub repository, certain steps may differ. The projects that come with the SDK
installer (found at `[INSTALL_DIR]\Device\Projects\`) are provided as zip-file
and will be copied into the e² Studio workspace. The projects downloaded or
cloned from the
[AFBR-S50 GitHub repository](https://github.com/Broadcom/AFBR-S50-API) are
already unpacked an must be imported but not copied into the e² Studio workspace
(otherwise the project references may become invalid).

In order to run provided project, execute the following steps:

## Install Renesas e² Studio IDE

Download and install the **e² Studio IDE**:

1. Go to <https://www.renesas.com/us/en/software-tool/e-studio>
2. Click on `Download` and register or sign in to download the installer.
3. Install the IDE.
4. Go to <https://www.renesas.com/us/en/software-tool/e2studio-support> for more information.

@note Please make sure the installation adds a recent version of the Renesas FSP
(Flexible Software Package). Otherwise, you may need to add it after the installation.
Go to <https://github.com/renesas/fsp/releases> and browse for the latest `FSP_Packs_v#.#.#.exe`.

## Import a Project

Import the project archive files:

1. Go to **e² Studio IDE** -> `Welcome Page`
2. Click on `Import existing projects`
3. Select your project for import:

    - If importing a project as zip file as provided by the **SDK installer**:

        1. Click on `Browse...` in the `Select archive file` section
        2. Browse to `[INSTALL_DIR]\Device\Projects\` (default is
           `C:\Program Files (x86)\Broadcom\AFBR-S50 SDK\Device\Projects\`)
        3. Select the required project archive (e.g.
           `AFBR_S50_Example_RA4M2.zip`) and click `Open`

    - If importing a project from the local directory as provided by
      cloning/downloading the **GitHub repository**:

        1. Click on `Browse...` in the `Select root directory` section
        2. Browse to `[REPOSITORY_ROOT]\Projects\e2studio\`
        3. Select the required project root folder (e.g.
           `AFBR_S50_Example_RA4M2`) and click `Select Folder`
        4. **IMPORTANT**: Click `Next`and disable the check at
           `Copy projects into Workspace`. (Otherwise, the references to the
           source files will be messed up!)
            @image html 3_9_import_project.PNG "Fig. 3.9: Import Project to e² Studio" width=400px
            @image latex 3_9_import_project.PNG "Fig. 3.9: Import Project to e² Studio"

4. Click `Finish`

## Build the Project

1. Go to **e² Studio IDE** -> `Project Explorer` and select the project to
   build.
2. Go to `Quickstart Panel` and click on `Build`

@image html 3_10_build_project.PNG "Fig. 3.10: Build the project on e² Studio." width=800px
@image latex 3_10_build_project.PNG "Fig. 3.10: Build the project on e² Studio."

\note If the build fails and the IDE complains about missing definitions or files,
make sure that the e2 Studio and FSP installations are not broken. This seems to
randomly happen when the FSP is updated to a newer version. In that case, completely
remove the Renesas e2 Studio and all related stuff (e.g. uninstall everything label
with Renesas) and reinstall the latest e2 Studio IDE and FSP package as described above.

## Debug the Project with a SWD debugger

The **Renesas RA4M2** on the @ref reference_board can be debugged with any SWD
debug probe supported by the **e² Studio**. Please refer to the **e² Studio**
documentation for more information. The following demonstrate the steps using a
**J-Link SWD Debug Probe**:

1. Connect the debug probe via SWD cable to the debug probe on the reference board.
2. Select the project to debug in the `Project Explorer` of the `Renesas e² Studio IDE`.
3. Select `Run` > `Debug Configurations...`.
4. Select `Renesas GDB Hardware Debugging` and click the `New Configuration` Button.
5. In the new configuration, go to `Debugger`
6. In `Debug Hardware` select `J-Link ARM`
7. In `Target Device` select `R7FA4M2AD`
8. Click `Apply` and `Debug`
    @image html 3_11_debug_config.PNG "Fig. 3.11: Debug Configuration." width=800px
    @image latex 3_11_debug_config.PNG "Fig. 3.11: Debug Configuration."
9. The program breaks at the `Reset_Handler()`. Click on `Resume` and
   program breaks at the `main()`, hit `Resume` and the program runs.
    @image html 3_12_start_program.PNG "Fig. 3.12: Resume the halted program." width=800px
    @image latex 3_12_start_program.PNG "Fig. 3.12: Resume the halted program."

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
