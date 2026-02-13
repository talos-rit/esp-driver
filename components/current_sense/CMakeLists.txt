idf_component_register(
  SRCS "ads1015.c"
  INCLUDE_DIRS "include"
  REQUIRES ads111x i2cdev
)