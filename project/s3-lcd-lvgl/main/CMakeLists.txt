

file(GLOB SRC_FILES 
)

file(GLOB_RECURSE SRC_FILES_RECURSE
  ap/*.c
  ap/*.cpp
)

set(requires 
  "esp32-s3-lcd-hal"
  "esp-dsp"
)

idf_component_register(
  SRCS "main.cpp"
  
  SRCS ${SRC_FILES}
  SRCS ${SRC_FILES_RECURSE}
  
  INCLUDE_DIRS ""
  INCLUDE_DIRS "ap"

  REQUIRES "${requires}"
  )
