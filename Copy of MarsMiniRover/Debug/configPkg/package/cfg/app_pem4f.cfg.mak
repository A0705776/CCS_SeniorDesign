# invoke SourceDir generated makefile for app.pem4f
app.pem4f: .libraries,app.pem4f
.libraries,app.pem4f: package/cfg/app_pem4f.xdl
	$(MAKE) -f C:\Users\ALAMIN~1\DOCUME~1\CCS_SE~1\COPYOF~1/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\ALAMIN~1\DOCUME~1\CCS_SE~1\COPYOF~1/src/makefile.libs clean

