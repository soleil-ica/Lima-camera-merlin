import PyTango
import numpy
import time

dev=PyTango.DeviceProxy('merlin/tango/1')
lima=PyTango.DeviceProxy('limaccd/tango/3')
tfg1=PyTango.DeviceProxy('tfg1/tango/1')

nframes = 4;
exp_time = 5.0;

# do acquisition
for i in range(1):
    lima.write_attribute("acq_nb_frames", nframes)
    lima.write_attribute("latency_time",0.01)
    lima.write_attribute("acq_expo_time", exp_time)
    lima.write_attribute("acq_trigger_mode", "EXTERNAL_GATE")
#    lima.write_attribute("acq_trigger_mode", "INTERNAL_TRIGGER")

    lima.command_inout("prepareAcq")
    lima.command_inout("startAcq")
    tfg1.command_inout("start")
    time.sleep(1)
    while lima.read_attribute("acq_status").value == "Running" :
        time.sleep(1)

print "Acquisition complete"
