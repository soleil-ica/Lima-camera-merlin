import PyTango
import numpy
import time

dev=PyTango.DeviceProxy('merlin/tango/1')
print "Software  version     :", dev.read_attribute("softwareversion").value

nframes = 10
exp_time = 2.0
allChannels = -1

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
lima.write_attribute("acq_expo_time",exp_time)
lima.write_attribute("acq_trigger_mode", "INTERNAL_TRIGGER")
lima.command_inout("prepareAcq")
lima.command_inout("startAcq")

while dev.read_attribute("acqRunning").value :
    time.sleep(0.5)

time.sleep(5)
