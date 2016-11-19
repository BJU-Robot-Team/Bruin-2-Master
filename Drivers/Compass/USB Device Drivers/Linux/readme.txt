CP210x Driver and Linux
 
The CP210x driver is distributed with 2.6 series kernels >=2.6.12. 
These can be downloaded in source form from http://www.kernel.org

The CP210x USB to Serial bridge is used on the OS5000-U* devices to
provide our virtual serial port.

Linux Project History:  
    http://www.etheus.net/CP210x_Linux_Driver#CP210x_Driver_Download

Silabs Information:

CP210x Linux VCP Driver v3.0 Release Notes
Copyright (C) 2008 Silicon Laboratories, Inc.

This release contains the following components:

	cp210x-3.0.0-1.spec - spec file for the driver build
	makerpm - makerpm file to build the RPM of the driver
	RELEASE-NOTES - This file
	INSTALL - installation instructions
	PACKAGE-LIST - list of files in the package
	REPORTING-BUGS - list of all known issues and limitations
	README - Revision history and release date file
	COPYING - GPL License
	cp210x\cp210x.c - Driver source file
	cp210x\cp210x.h - Driver header file
	cp210x\Rules.make - Rules for the 2.4 kernel make
	cp210x\Makefile24 - Make file for 2.4 Kernel
	cp210x\Makefile26 - Make file for 2.6 Kernel
	cp210x\configure - Configure file for the build
	cp210x\installmod - Install module script
	cp210x\.rpmmacros - rpmmacros file
	rpm\brp-java-repack-jars - Files needed for the rpm build
	rpm\brp-python-bytecompile - Files needed for the rpm build
	rpm\check-rpaths - Files needed for the rpm build
	rpm\check-rpaths-worker - Files needed for the rpm build


Driver Installation
-------------------

	1. Extract the source package into the home directory by running:
		gzip -cd cp210x-x.y.z-i.tar.gz | tar xvf -
	
	2. Modify line 17 of the ./cp210x/Rules.make file to set KENELDIR to point to 
	the local source directory. (2.4 kernel only)
	
	3. Modify line 98 of the ./cp210x/cp210x.h file to set the include directory to
	point to the local source directory's copy of usb-serial.h. (2.4 kernel only)
	
	4. Use the following command to build the rpm:
		./makerpm 
	It will be generated in:
		/var/tmp/silabs/rpmbuild/RPM/i386/cp210x-x.y.z-i.rpm
	where x.y.z-i is the current release number.
	
	5. Browse to the directory above containing the new rpm and run:
		rpm -Uvh cp210x-x.y.z-i.rpm
	where x.y.z-i is the current release number.
	
	6. This will install the driver if it is not already installed and up to date.
	Reboot the computer to complete the installation. (optional)


Known Issues and Limitations
----------------------------

2.6 Kernel
	- makerpm - only supports builds for the running kernel.
	- .spec - dependencies are broken.

2.4 Kernel
	- Built to support only 2.4.36 (the last release of 2.4 at the time of
	development).
	- makerpm - may require a root password if the following files are not in the
	/usr/lib/rpm directory:
		brp-java-repack-jars
		brp-python-bytecompile
		check-rpaths
		check-rpaths-worker
	- .spec - dependencies are broken.
	- makerpm - only supports builds for the kernel linked to by /usr/src/linux-2.4
	is hard coded too. KERNELDIR is defined in Rules.make on line 17:
		KERNELDIR = /home/user/linux-2.x.y
	- cp210x.h - path to usb-serial.h is hardcoded.


Release Dates
-------------
	CP210x Linux VCP Driver v3.0 - June 27, 2008


CP210x Linux Driver Revision History
--------------------------------------

Version 3.0

	Supports 2.4 and 2.6 through a single release
	
	Merged 2.4 and 2.6 drivers into a single package that can build
	and RPM for the target Kernel
	
Version 2.0

	Supports 2.4 and 2.6 Kernels through separate releases
	
	Support for 2.6 Kernel added
	
Version 1.1

	Supports 2.4 Kernel only
	
	Support added for higher baud-rates (460800baud and 921600baud)

Version 0.81b

	Supports 2.4 Kernel only
	
	This version fixes an issue which caused a machine crash when 
	disconnecting from a modem.

Version 0.81

	Supports 2.4 Kernel only
	
	Initial Release



