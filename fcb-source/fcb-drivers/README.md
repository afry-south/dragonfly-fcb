This directory contains files that have been first copied from

fcb/STM32Cube_FW_F3_V1.1.0 directory

and then modified for FCB purposes. This means that
1) when files in this directory having the same name as in the original directory, the project should use the one in this diretory.
2) The original(ish) file is in the STM32Cube_FW_F3_V1.1.0 directory but should not be used to compile the project.
3) stm32f3_discovery_gyroscope.c was used but then superseded by functions inside l3gd20.c
   - this reduces extra function calls and makes following call stack easier.
