import PyTango
import numpy
import time

dev=PyTango.DeviceProxy('merlin/tango/1')
print "Software  version     :", dev.read_attribute("softwareversion").value

nframes = 10 
exp_time = 0.1

lima=PyTango.DeviceProxy('limaccd/tango/3')
# do not change the order of the saving attributes!
lima.write_attribute("saving_directory","/home/grm84/data")
lima.write_attribute("saving_format","HDF5")
lima.write_attribute("saving_overwrite_policy","Overwrite")
lima.write_attribute("saving_suffix", ".hdf")
lima.write_attribute("saving_prefix","merlin_")
lima.write_attribute("saving_mode","AUTO_FRAME")
lima.write_attribute("saving_frame_per_file", nframes)

#dev.write_attribute("colourMode","monochrome");
#dev.write_attribute("colourMode","colour");

#dev.write_attribute("counter","counter0");
#dev.write_attribute("counter","counter1");
#dev.write_attribute("counter","both");

# do acquisition
lima.write_attribute("acq_trigger_mode", "INTERNAL_TRIGGER")
lima.write_attribute("acq_nb_frames",nframes)
lima.write_attribute("latency_time",0.01)
lima.write_attribute("acq_expo_time",exp_time)

for i in range(10):
    lima.command_inout("prepareAcq")
    lima.command_inout("startAcq")

    while dev.read_attribute("acqRunning").value :
        time.sleep(.2)
        print "running"

    print "Completed ", i, " - idle"
    time.sleep(2)

print "Done"
