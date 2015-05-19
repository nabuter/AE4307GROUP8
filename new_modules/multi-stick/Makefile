# =========================================================================
#       item            : template Makefile dueca module
#       made by         : René van Paassen
#       date            : 010320
# =========================================================================

# =========================================================================
#
# EDIT THIS SECTION TO INSERT YOUR SOURCE FILES
#
# =========================================================================

# if needed, add the path to extra header files you need
EXTRA_INCLUDES= 

# add all c++-files here (.cxx files), except the automatically generated 
# ones. The line that is in there now simply finds all .cxx files. Change 
# to : CPPSRC = File1.cxx File2.cxx
# if you don't want that behaviour
CPPSRC := 	$(shell ls *.cxx 2>/dev/null)

# and all c-files here (.c files)
CSRC :=		$(shell ls *.c 2>/dev/null)

# add all the files with the definitions of the generated-types (.dco files)
DUECATYPES := 	$(shell ls *.dco 2>/dev/null)

# the rest is handled by generic rules
TAILFILE :=	$(shell dueca-config --path-datafiles)/data/MakefileTail.mod
include $(TAILFILE)


