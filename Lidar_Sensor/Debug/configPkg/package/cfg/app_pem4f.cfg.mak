# invoke SourceDir generated makefile for app.pem4f
app.pem4f: .libraries,app.pem4f
.libraries,app.pem4f: package/cfg/app_pem4f.xdl
	$(MAKE) -f C:\Users\Alamin_Bird\Documents\CCS_SeniorDesign\Lidar_Sensor/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\Alamin_Bird\Documents\CCS_SeniorDesign\Lidar_Sensor/src/makefile.libs clean

