############################################################################
#
# The main Makefile for the Nearest Neighbor Library for Motion Planning
#
############################################################################

all:
	cd src ; make all
#	cd test ; make all

#-----------------------------------------------------------------------------
# Remove .o files and core files
#-----------------------------------------------------------------------------
clean:
	cd src ; make clean
	cd test ; make clean

#-----------------------------------------------------------------------------
# Remove everthing that can be remade
#-----------------------------------------------------------------------------
realclean: 
	-rm -f lib/*.a lib/*.so
	cd src ; make realclean
	cd test ; make realclean
