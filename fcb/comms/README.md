
# Generate source & header files from the .proto file

Put "fcb\nanopb-0.3.3-windows-x86\generator-bin¨ from this cloned repository into
your PATH.

Cd to the directory of this README file, then execute:

protoc  --nanopb_out=. fcb-fsm.proto

# Notes
* For more on the Protocol Buffers message de/serialisation utility, see here: https://developers.google.com/protocol-buffers/
