#####################################################################
# use this script to load your code via a STM Link connected to the
# raspi in mowgli
#####################################################################
Import("env")

host = env.GetProjectOption("custom_mowgli_host")
user = env.GetProjectOption("custom_mowgli_user")


def mytarget_callback(*args, **kwargs):
    print("Hello PlatformIO!")
    env.Execute("scp \"$BUILD_DIR/${PROGNAME}.bin\" " + user + "@" + host + ":/tmp/firmware.bin")        
    env.Execute("scp ./remote_upload/*.cfg " + user + "@" + host + ":/tmp")    
    env.Execute("ssh " + user + "@" + host + " /usr/bin/openocd -f /tmp/yardforce500.cfg  -f /tmp/prog.cfg")
         
env.AddCustomTarget(
    "Mowgli Remote Upload",
    "$BUILD_DIR/${PROGNAME}.bin",
    mytarget_callback
)
