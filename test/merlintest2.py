import PyTango
import numpy
import time
import random

dev=PyTango.DeviceProxy('merlin/tango/1')
print "Software  version     :", dev.read_attribute("softwareversion").value

nframes = 50
exp_time = 0.01

lima=PyTango.DeviceProxy('limaccd/tango/3')
# do not change the order of the saving attributes!
# lima.write_attribute("saving_directory","/home/grm84/data")
# lima.write_attribute("saving_format","HDF5")
# lima.write_attribute("saving_overwrite_policy","Overwrite")
# lima.write_attribute("saving_suffix", ".hdf")
# lima.write_attribute("saving_prefix","merlin_")
# lima.write_attribute("saving_mode","AUTO_FRAME")
# lima.write_attribute("saving_frame_per_file", nframes)

dev.write_attribute("triggerStartType","INTERNAL");

for i in range(2):
    # do acquisition
    lima.write_attribute("acq_nb_frames",nframes)
    lima.write_attribute("latency_time",0.01)
    lima.write_attribute("acq_expo_time",exp_time)
    lima.write_attribute("acq_trigger_mode", "INTERNAL_TRIGGER")

    lima.command_inout("prepareAcq")
    start = time.time()
    lima.command_inout("startAcq")
    print "Running "
    while lima.read_attribute("acq_status").value == "Running" :
        time.sleep(.2)

    print "Completed run in ", time.time()-start, "secs"


print "Done"
