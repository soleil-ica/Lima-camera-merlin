import PyTango
import time

dev_merlin = PyTango.DeviceProxy('merlin/merlin/merlin.1')
dev_cpt1 = PyTango.DeviceProxy('labodt/cpt-det/pulsecounting-det')

dev_merlin.stop()
dev_merlin.nbframes = 1

# a loop with a sleep after the snap
for i in range(50):
    print 'iteration:',i
    dev_merlin.prepare()
    dev_merlin.snap()
    # it works fine with the sleep
    time.sleep(0.1)
    dev_cpt1.start()
    while dev_merlin.state() in [PyTango.DevState.RUNNING]:
        dev_merlin.state()
        time.sleep(1)

# No sleep after the snap
# dev_merlin.prepare()
# dev_merlin.snap()
# dev_cpt1.start()