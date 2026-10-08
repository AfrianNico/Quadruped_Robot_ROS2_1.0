Import("env")

lmicroros = env.subst("$PROJECT_DIR/.pio/libdeps/$PIOENV/micro_ros_arduino/src/esp32/libmicroros.a")
env.Append(LIBS=[env.File(lmicroros)])
