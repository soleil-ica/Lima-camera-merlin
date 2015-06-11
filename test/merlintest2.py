import PyTango
import numpy
import time

dev=PyTango.DeviceProxy('merlin/tango/1')
print "Software  version     :", dev.read_attribute("softwareversion").value

nframes = 5
exp_time = 0.1

lima=PyTango.DeviceProxy('limaccd/tango/3')
# do not change the order of the saving attributes!
lima.write_attribute("saving_directory","/home/grm84/data")
lima.write_attribute("saving_format","HDF5")
lima.write_attribute("saving_overwrite_policy","Overwrite")
lima.write_attribute("saving_suffix", ".hdf")
lima.write_attribute("saving_prefix","merlin_")
lima.write_attribute("saving_mode","AUTO_FRAME")
lima.write_attribute("saving_frames_per_file", nframes)

# do acquisition
lima.write_attribute("acq_nb_frames",nframes)
lima.write_attribute("latency_time",0.01)
lima.write_attribute("acq_expo_time",exp_time)
lima.write_attribute("acq_trigger_mode", "INTERNAL_TRIGGER")

for i in range(100):
    lima.command_inout("prepareAcq")
    lima.command_inout("startAcq")

#    while dev.read_attribute("acqRunning").value :
    time.sleep(.3)
#        print "running"
    lima.command_inout("stopAcq")
       

    print "Completed ", i, " - idle"
    time.sleep(2)

print "Done"
#lima.command_inout("stopAcq")

#time.sleep(5)
