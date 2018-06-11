.. _camera-merlin:

Merlin camera
-------------

.. image:: merlin.jpg
   :scale: 20 %


Introduction
````````````

The Merlin Medipix3Rx Quad Readout detector system from Diamond Light Source Ltd is a photon counting soild state pixel detector with a silicon sensor.

The Lima module has only been tested  in a 2 x 2 configuration, but is available in a 4 x 1 configuration

There is extensive documentation :ref: `Merlin_and_Medipix3_Documentation_v0.7.pdf`

Prerequisite
````````````
The Merlin detector system is based on a National Instruments FlexRIO PXI FPGA system.
It incorporates an embedded PC running Windows with a LabView graphical user interface, incorporating a socket server, which this plugin communicates with.
This program must be running prior to starting Lima.

Installation & Module configuration
```````````````````````````````````

Follow the generic instructions in :ref:`build_installation`. If using CMake directly, add the following flag:

.. code-block:: sh

 -DLIMACAMERA_MERLIN=true

For the Tango server installation, refers to :ref:`tango_installation`.

Initialisation and Capabilities
```````````````````````````````

Implementing a new plugin for new detector is driven by the LIMA framework but the developer has some freedoms to choose which standard and specific features will be made available. This section is supposed to give you good knowledge regarding camera features within the LIMA framework.

Camera initialisation
.....................

The camera has to be initialized using the :cpp:class:`MerlinCamera` class. The constructor requires the hostname of the detector system.

Std capabilities
................

This plugin has been implemented with the mandatory capabilites, with some limitations due to the camera server implementation.

* HwDetInfo

 The detector is set to full image size at startup which means a binning of 1x1. There is no hardware binning

* HwSync

  The supported trigger modes are:

   - IntTrig
   - IntTrigMult
   - ExtTrigSingle
   - ExtTrigMult

Testing
```````

This is a simple python test program:

.. code-block:: python

  from Lima import Merlin
  from Lima import Core
  import time

  camera = Merlin.Camera('<hostname>')
  interface = Merlin.Interface(camera)
  control = Core.CtControl(interface)

  acq = control.acquisition()

  # check its OK
  print camera.getDetectorType()
  print camera.getDetectorModel()
  print camera.getSoftwareVersion()

  nframes=5
  acqtime=3.0
  # setting new file parameters and autosaving mode
  saving=control.saving()

  saving.setDirectory("/home/grm84/data")
  saving.setFramesPerFile(nframes)
  saving.setFormat(Core.CtSaving.HDF5)
  saving.setPrefix("merlin_")
  saving.setSuffix(".hdf")
  saving.setSavingMode(Core.CtSaving.AutoFrame)
  saving.setOverwritePolicy(Core.CtSaving.Append)

  # do acquisition
  acq=control.acquisition()
  acq.setAcqExpoTime(acqtime)
  acq.setAcqNbFrames(nframes)

  control.prepareAcq()
  control.startAcq()

  # wait for last image (#4) ready
  lastimg = control.getStatus().ImageCounters.LastImageReady
  while lastimg !=nframes-1:
    time.sleep(0.01)
    lastimg = control.getStatus().ImageCounters.LastImageReady

  # read the first image
  im0 = control.ReadImage(0)
