import PyTango
import numpy
import time

dev=PyTango.DeviceProxy('merlin/tango/1')
lima=PyTango.DeviceProxy('limaccd/tango/3')
tfg1=PyTango.DeviceProxy('tfg1/tango/1')

nframes = 3;
exp_time = 5.0;

print "Software  version     :", dev.read_attribute("softwareversion").value

dev.write_attribute("triggerStartType","INTERNAL");
dev.write_attribute("triggerStopType","INTERNAL");

# do acquisition
for i in range(3):
    lima.write_attribute("acq_nb_frames", nframes)
    lima.write_attribute("latency_time",0.01)
    lima.write_attribute("acq_expo_time", exp_time)
    lima.write_attribute("acq_trigger_mode", "INTERNAL_TRIGGER")

    i = 0
    print "======================",lima.read_attribute("acq_status").value
    lima.command_inout("prepareAcq")
    lima.command_inout("startAcq")
    while lima.read_attribute("acq_status").value == "Running" :
        time.sleep(1)
        i = i + 1
        if i > 2:
            lima.command_inout("stopAcq")
            break
        print "Acquisition complete"

    lima.command_inout("stopAcq")
    while lima.read_attribute("acq_status").value == "Running" :
        continue
    lima.command_inout("prepareAcq")
    lima.command_inout("startAcq")
    while lima.read_attribute("acq_status").value == "Running" :
        time.sleep(1)

print "Acquisition complete"
