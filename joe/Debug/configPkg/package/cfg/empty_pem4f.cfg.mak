# invoke SourceDir generated makefile for empty.pem4f
empty.pem4f: .libraries,empty.pem4f
.libraries,empty.pem4f: package/cfg/empty_pem4f.xdl
	$(MAKE) -f C:\Users\Alamin_Bird\Documents\CCS_SeniorDesign\joe/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\Alamin_Bird\Documents\CCS_SeniorDesign\joe/src/makefile.libs clean

