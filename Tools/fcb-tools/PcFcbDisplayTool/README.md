# README for PcFcbTool.py

## Python installation
* This tool was developed to run on a Windows 7 workstation.
* General python installation:See Dragonfly-FCB Wiki page "ToolTipsTricks", section "Python" and follow installation instructions.
* Open `cmd.exe`
* Install library dependencies: `C:\INSTALLATION_PATH\python.exe -m pip install pyserial numpy protobuf pyside pyqtgraph`
 * If the `error: Microsoft Visual C++ 9.0 is required (Unable to find vcvarsall.bat)` occurs, Get it from http://aka.ms/vcpython27
* Run `C:\INSTALLATION_PATH\python -m pip install pyside pyqtgraph`
* now it should be possible to run `C:\INSTALLATION_PATH\python.exe PcFcbTool.py 100 60` in `cmd.exe`.

## PcFcbTool usage
* Connect the FCB to the Windows 7 PC using a micro USB cable.
* run `python PcFcbTool.py 200 30` to receive a graph which is sampled every 200 ms for 30 seconds.
* Close the graph window to exit the program.
* use the `--help` option to get usage instructions.

## Developing PcFcbDisplayTool
* In order to regenerate `dragonfly_fcb_pb2.py`:
 * Download `protoc-3.0.0-beta-2-win32.zip` from: https://github.com/google/protobuf/releases extract `protoc.exe` and make sure it is in the `Path` of `cmd.exe`
 * Then run: `protoc --plugin=generator\protoc-gen-nanopb.bat --proto_path=REPO_PATH\fcb-source\communication\protobuf\ REPO_PATH\fcb-source\communication\protobuf\dragonfly_fcb.proto  --python_out=REPO_PATH\Tools\fcb-tools\PcFcbDisplayTool

## To Do
 * use `argparse` for CLI options parsing
