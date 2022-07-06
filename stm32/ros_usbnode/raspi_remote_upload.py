#####################################################################
# use this script to load your code via a STM Link connected to the
# raspi in mowgli
#####################################################################
Import("env")

host = env.GetProjectOption("custom_mowgli_host")
user = env.GetProjectOption("custom_mowgli_user")
sshkeyfile = env.GetProjectOption("sshkeyfile")


def mytarget_callback(*args, **kwargs):
    print("Hello PlatformIO!")
<<<<<<< HEAD
    env.Execute("scp  -i " + sshkeyfile + "  \"$BUILD_DIR/${PROGNAME}.bin\" " + user + "@" + host + ":/tmp/firmware.bin")        
    env.Execute("scp  -i " + sshkeyfile + " ./remote_upload/*.cfg " + user + "@" + host + ":/tmp")    
    env.Execute("ssh  -i " + sshkeyfile + " " + user + "@" + host + " /usr/bin/openocd -f /tmp/yardforce500.cfg  -f /tmp/prog.cfg")
    env.Execute("ssh  -i " + sshkeyfile + " " + user + "@" + host + " \"sleep 5; sudo systemctl restart rosserial\"")
=======
    env.Execute("scp \"$BUILD_DIR/${PROGNAME}.bin\" " + user + "@" + host + ":/tmp/firmware.bin")        
    env.Execute("scp ./remote_upload/*.cfg " + user + "@" + host + ":/tmp")    
    env.Execute("ssh " + user + "@" + host + " sudo /usr/bin/openocd -f /tmp/yardforce500.cfg  -f /tmp/prog.cfg")
    env.Execute("ssh " + user + "@" + host + " \"sleep 5; sudo systemctl restart rosserial\"")
>>>>>>> 79f0f1996556ecac8f58bf8a46a34640c9b69284
         
env.AddCustomTarget(
    "Mowgli Remote Upload",
    "$BUILD_DIR/${PROGNAME}.bin",
    mytarget_callback
)
