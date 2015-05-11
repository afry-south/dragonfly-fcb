CFLAGS += -I. -I/cygdrive/h/project/dragonfly/tools/nanopb-0.3.3-windows-x86
CFLAGS += -Wall -Werror

PROTO_FILES := fcb-fsm.proto

PROTO_SRC := $(PROTO_FILES:.proto=.pb.c)
PROTO_HDR := $(PROTO_FILES:.proto=.pb.h)




$(PROTO_SRC) $(PROTO_HDR) : $(PROTO_FILES)
	myprotoc  --nanopb_out=. $<

# use this for workstation evaluation by uncommenting
# a main.c file using the generated result must be
# created first.
#
# main.c: $(PROTO_HDR)
#
# fcb-fsm : main.c pb_encode.c pb_decode.c pb_common.c $(PROTO_SRC)
#	gcc $(CFLAGS) -o $@ $^
