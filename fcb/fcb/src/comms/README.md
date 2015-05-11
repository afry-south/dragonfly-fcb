
Surf to http://koti.kapsi.fi/jpa/nanopb/

Get nanopb-0.3.3-windows-x86.zip from "Stable Releases" and unzip it.

Put generator-bin/ from the zipped directory into your PATH.

Cd to the directory of this file, then execute:

protoc  --nanopb_out=. fcb-fsm.proto
