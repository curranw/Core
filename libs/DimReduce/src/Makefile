include ../Make.inc

##########################################################################
# The rest should not need to be changed


#Define Object dependencies
MATVEC_BASE = MatVec.o MatSym.o MatTri.o MatVecDecomp.o
LLE_BASE = LLE.o
RLLE_BASE = RLLE.o IRWPCA.o HLLE.o
DIMREDUCE_OBJECTS = DimReduce.o fitstools.o Data.o $(LLE_BASE) $(MATVEC_BASE) $(RLLE_BASE) 

PCA_OBJECTS = PCA.o fitstools.o $(MATVEC_BASE)

MatVec_test_objects = MatVec_test.o $(MATVEC_BASE)
fits_stat_objects = fits_stat.o fitstools.o MatVec.o

#Define all executables
#all :  DimReduce areig_test MatVec_test
all: DimReduce PCA fits_stat MatVec_test PCA_test

#Dependencies and compilation of each executable

DimReduce : $(DIMREDUCE_OBJECTS)
	$(GCC) $^ -o $@ $(CFLAGS) $(FITS_CFLAGS)

MatVec_test: $(MatVec_test_objects)
	$(GCC) $^ -o $@ $(CFLAGS)

fits_stat: $(fits_stat_objects)
	$(GCC) $^ -o $@ $(FITS_CFLAGS) $(CFLAGS)

PCA_test: $(PCA_OBJECTS) PCA_test.o
	$(GCC) $^ -o $@ $(FITS_CFLAGS) $(CFLAGS)

PCA: $(PCA_OBJECTS) PCA_main.o
	$(GCC) $^ -o $@ $(FITS_CFLAGS) $(CFLAGS)

#object compilation
.cpp.o:
	$(GCC) -c $< -o $@ $(GCC_INC) $(FITS_INC)

.f.o:
	$(F77) -c $< -o $@ $(F77_INC)

clean :
	rm -f *.o
	rm -f *~ .*~

depend:
	makedepend *.cpp $(GCC_DEP)

# below here are the automatically generated dependencies
# DO NOT DELETE

Data.o: ../include/Data.h ../include/MatVec.h ../include/MatVecExcept.h
Data.o: ../include/MatVecBase.h ../include/blas.h
Data.o: ../include/DimReduceExcept.h
DimReduce.o: ../include/LLE.h ../include/MatVec.h ../include/MatVecExcept.h
DimReduce.o: ../include/MatVecBase.h ../include/blas.h ../include/fitstools.h
DimReduce.o: /astro/users/vanderplas/lib/cfitsio/include/fitsio.h
DimReduce.o: /usr/include/stdio.h /usr/include/features.h
DimReduce.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
DimReduce.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h
DimReduce.o: /usr/include/bits/types.h /usr/include/bits/typesizes.h
DimReduce.o: /usr/include/libio.h /usr/include/_G_config.h
DimReduce.o: /usr/include/wchar.h /usr/include/bits/wchar.h
DimReduce.o: /usr/include/gconv.h /usr/include/bits/stdio_lim.h
DimReduce.o: /usr/include/bits/sys_errlist.h /usr/include/stdlib.h
DimReduce.o: /usr/include/sys/types.h /usr/include/time.h
DimReduce.o: /usr/include/endian.h /usr/include/bits/endian.h
DimReduce.o: /usr/include/sys/select.h /usr/include/bits/select.h
DimReduce.o: /usr/include/bits/sigset.h /usr/include/bits/time.h
DimReduce.o: /usr/include/sys/sysmacros.h /usr/include/bits/pthreadtypes.h
DimReduce.o: /usr/include/alloca.h /usr/include/limits.h
DimReduce.o: /usr/include/bits/posix1_lim.h /usr/include/bits/local_lim.h
DimReduce.o: /usr/include/linux/limits.h /usr/include/bits/posix2_lim.h
DimReduce.o: /astro/users/vanderplas/lib/cfitsio/include/longnam.h
DimReduce.o: ../include/HLLE.h ../include/RLLE.h /usr/include/math.h
DimReduce.o: /usr/include/bits/huge_val.h /usr/include/bits/mathdef.h
DimReduce.o: /usr/include/bits/mathcalls.h ../include/doc.h
DimReduce.o: /usr/include/string.h
HLLE.o: ../include/HLLE.h ../include/MatVec.h ../include/MatVecExcept.h
HLLE.o: ../include/MatVecBase.h ../include/blas.h ../include/LLE.h
HLLE.o: ../include/fitstools.h
HLLE.o: /astro/users/vanderplas/lib/cfitsio/include/fitsio.h
HLLE.o: /usr/include/stdio.h /usr/include/features.h /usr/include/sys/cdefs.h
HLLE.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
HLLE.o: /usr/include/gnu/stubs-64.h /usr/include/bits/types.h
HLLE.o: /usr/include/bits/typesizes.h /usr/include/libio.h
HLLE.o: /usr/include/_G_config.h /usr/include/wchar.h
HLLE.o: /usr/include/bits/wchar.h /usr/include/gconv.h
HLLE.o: /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h
HLLE.o: /usr/include/stdlib.h /usr/include/sys/types.h /usr/include/time.h
HLLE.o: /usr/include/endian.h /usr/include/bits/endian.h
HLLE.o: /usr/include/sys/select.h /usr/include/bits/select.h
HLLE.o: /usr/include/bits/sigset.h /usr/include/bits/time.h
HLLE.o: /usr/include/sys/sysmacros.h /usr/include/bits/pthreadtypes.h
HLLE.o: /usr/include/alloca.h /usr/include/limits.h
HLLE.o: /usr/include/bits/posix1_lim.h /usr/include/bits/local_lim.h
HLLE.o: /usr/include/linux/limits.h /usr/include/bits/posix2_lim.h
HLLE.o: /astro/users/vanderplas/lib/cfitsio/include/longnam.h
HLLE.o: ../include/MatSym.h ../include/MatVecDecomp.h
HLLE.o: ../include/DimReduceExcept.h ../include/argsort.h /usr/include/math.h
HLLE.o: /usr/include/bits/huge_val.h /usr/include/bits/mathdef.h
HLLE.o: /usr/include/bits/mathcalls.h
IRWPCA.o: ../include/IRWPCA.h ../include/MatVec.h ../include/MatVecExcept.h
IRWPCA.o: ../include/MatVecBase.h ../include/blas.h ../include/MatSym.h
IRWPCA.o: ../include/MatVecDecomp.h ../include/DimReduceExcept.h
IRWPCA.o: /usr/include/math.h /usr/include/features.h
IRWPCA.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
IRWPCA.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h
IRWPCA.o: /usr/include/bits/huge_val.h /usr/include/bits/mathdef.h
IRWPCA.o: /usr/include/bits/mathcalls.h
LLE.o: ../include/LLE.h ../include/MatVec.h ../include/MatVecExcept.h
LLE.o: ../include/MatVecBase.h ../include/blas.h ../include/fitstools.h
LLE.o: /astro/users/vanderplas/lib/cfitsio/include/fitsio.h
LLE.o: /usr/include/stdio.h /usr/include/features.h /usr/include/sys/cdefs.h
LLE.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
LLE.o: /usr/include/gnu/stubs-64.h /usr/include/bits/types.h
LLE.o: /usr/include/bits/typesizes.h /usr/include/libio.h
LLE.o: /usr/include/_G_config.h /usr/include/wchar.h
LLE.o: /usr/include/bits/wchar.h /usr/include/gconv.h
LLE.o: /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h
LLE.o: /usr/include/stdlib.h /usr/include/sys/types.h /usr/include/time.h
LLE.o: /usr/include/endian.h /usr/include/bits/endian.h
LLE.o: /usr/include/sys/select.h /usr/include/bits/select.h
LLE.o: /usr/include/bits/sigset.h /usr/include/bits/time.h
LLE.o: /usr/include/sys/sysmacros.h /usr/include/bits/pthreadtypes.h
LLE.o: /usr/include/alloca.h /usr/include/limits.h
LLE.o: /usr/include/bits/posix1_lim.h /usr/include/bits/local_lim.h
LLE.o: /usr/include/linux/limits.h /usr/include/bits/posix2_lim.h
LLE.o: /astro/users/vanderplas/lib/cfitsio/include/longnam.h
LLE.o: ../include/DimReduceExcept.h ../include/MatSym.h
LLE.o: ../include/MatVecDecomp.h ../include/argsort.h ../include/Data.h
LLE.o: /usr/include/math.h /usr/include/bits/huge_val.h
LLE.o: /usr/include/bits/mathdef.h /usr/include/bits/mathcalls.h
MatSym.o: ../include/MatSym.h ../include/MatVecExcept.h
MatSym.o: ../include/MatVecBase.h ../include/blas.h ../include/MatVec.h
MatTri.o: ../include/MatTri.h ../include/MatVecExcept.h
MatTri.o: ../include/MatVecBase.h ../include/blas.h ../include/MatVec.h
MatTri_test.o: ../include/MatTri.h ../include/MatVecExcept.h
MatTri_test.o: ../include/MatVecBase.h ../include/blas.h ../include/MatVec.h
MatVec.o: ../include/MatVec.h ../include/MatVecExcept.h
MatVec.o: ../include/MatVecBase.h ../include/blas.h
MatVecDecomp.o: ../include/MatVec.h ../include/MatVecExcept.h
MatVecDecomp.o: ../include/MatVecBase.h ../include/blas.h ../include/lapack.h
MatVecDecomp.o: ../include/arpack.h ../include/MatVecDecomp.h
MatVecDecomp.o: ../include/MatSym.h
MatVec_test.o: ../include/MatVec.h ../include/MatVecExcept.h
MatVec_test.o: ../include/MatVecBase.h ../include/blas.h ../include/MatSym.h
MatVec_test.o: ../include/MatTri.h ../include/MatVecDecomp.h
MatVec_test.o: /usr/include/time.h /usr/include/bits/types.h
MatVec_test.o: /usr/include/features.h /usr/include/sys/cdefs.h
MatVec_test.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
MatVec_test.o: /usr/include/gnu/stubs-64.h /usr/include/bits/typesizes.h
MatVec_test.o: /usr/include/math.h /usr/include/bits/huge_val.h
MatVec_test.o: /usr/include/bits/mathdef.h /usr/include/bits/mathcalls.h
MatVec_test.o: /usr/include/stdlib.h /usr/include/sys/types.h
MatVec_test.o: /usr/include/endian.h /usr/include/bits/endian.h
MatVec_test.o: /usr/include/sys/select.h /usr/include/bits/select.h
MatVec_test.o: /usr/include/bits/sigset.h /usr/include/bits/time.h
MatVec_test.o: /usr/include/sys/sysmacros.h /usr/include/bits/pthreadtypes.h
MatVec_test.o: /usr/include/alloca.h
PCA.o: ../include/PCA.h ../include/MatVec.h ../include/MatVecExcept.h
PCA.o: ../include/MatVecBase.h ../include/blas.h ../include/fitstools.h
PCA.o: /astro/users/vanderplas/lib/cfitsio/include/fitsio.h
PCA.o: /usr/include/stdio.h /usr/include/features.h /usr/include/sys/cdefs.h
PCA.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
PCA.o: /usr/include/gnu/stubs-64.h /usr/include/bits/types.h
PCA.o: /usr/include/bits/typesizes.h /usr/include/libio.h
PCA.o: /usr/include/_G_config.h /usr/include/wchar.h
PCA.o: /usr/include/bits/wchar.h /usr/include/gconv.h
PCA.o: /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h
PCA.o: /usr/include/stdlib.h /usr/include/sys/types.h /usr/include/time.h
PCA.o: /usr/include/endian.h /usr/include/bits/endian.h
PCA.o: /usr/include/sys/select.h /usr/include/bits/select.h
PCA.o: /usr/include/bits/sigset.h /usr/include/bits/time.h
PCA.o: /usr/include/sys/sysmacros.h /usr/include/bits/pthreadtypes.h
PCA.o: /usr/include/alloca.h /usr/include/limits.h
PCA.o: /usr/include/bits/posix1_lim.h /usr/include/bits/local_lim.h
PCA.o: /usr/include/linux/limits.h /usr/include/bits/posix2_lim.h
PCA.o: /astro/users/vanderplas/lib/cfitsio/include/longnam.h
PCA.o: ../include/MatVecDecomp.h ../include/MatSym.h
PCA.o: ../include/DimReduceExcept.h /usr/include/math.h
PCA.o: /usr/include/bits/huge_val.h /usr/include/bits/mathdef.h
PCA.o: /usr/include/bits/mathcalls.h
PCA_main.o: ../include/PCA.h ../include/MatVec.h ../include/MatVecExcept.h
PCA_main.o: ../include/MatVecBase.h ../include/blas.h ../include/fitstools.h
PCA_main.o: /astro/users/vanderplas/lib/cfitsio/include/fitsio.h
PCA_main.o: /usr/include/stdio.h /usr/include/features.h
PCA_main.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
PCA_main.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h
PCA_main.o: /usr/include/bits/types.h /usr/include/bits/typesizes.h
PCA_main.o: /usr/include/libio.h /usr/include/_G_config.h
PCA_main.o: /usr/include/wchar.h /usr/include/bits/wchar.h
PCA_main.o: /usr/include/gconv.h /usr/include/bits/stdio_lim.h
PCA_main.o: /usr/include/bits/sys_errlist.h /usr/include/stdlib.h
PCA_main.o: /usr/include/sys/types.h /usr/include/time.h
PCA_main.o: /usr/include/endian.h /usr/include/bits/endian.h
PCA_main.o: /usr/include/sys/select.h /usr/include/bits/select.h
PCA_main.o: /usr/include/bits/sigset.h /usr/include/bits/time.h
PCA_main.o: /usr/include/sys/sysmacros.h /usr/include/bits/pthreadtypes.h
PCA_main.o: /usr/include/alloca.h /usr/include/limits.h
PCA_main.o: /usr/include/bits/posix1_lim.h /usr/include/bits/local_lim.h
PCA_main.o: /usr/include/linux/limits.h /usr/include/bits/posix2_lim.h
PCA_main.o: /astro/users/vanderplas/lib/cfitsio/include/longnam.h
PCA_test.o: ../include/PCA.h ../include/MatVec.h ../include/MatVecExcept.h
PCA_test.o: ../include/MatVecBase.h ../include/blas.h ../include/fitstools.h
PCA_test.o: /astro/users/vanderplas/lib/cfitsio/include/fitsio.h
PCA_test.o: /usr/include/stdio.h /usr/include/features.h
PCA_test.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
PCA_test.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h
PCA_test.o: /usr/include/bits/types.h /usr/include/bits/typesizes.h
PCA_test.o: /usr/include/libio.h /usr/include/_G_config.h
PCA_test.o: /usr/include/wchar.h /usr/include/bits/wchar.h
PCA_test.o: /usr/include/gconv.h /usr/include/bits/stdio_lim.h
PCA_test.o: /usr/include/bits/sys_errlist.h /usr/include/stdlib.h
PCA_test.o: /usr/include/sys/types.h /usr/include/time.h
PCA_test.o: /usr/include/endian.h /usr/include/bits/endian.h
PCA_test.o: /usr/include/sys/select.h /usr/include/bits/select.h
PCA_test.o: /usr/include/bits/sigset.h /usr/include/bits/time.h
PCA_test.o: /usr/include/sys/sysmacros.h /usr/include/bits/pthreadtypes.h
PCA_test.o: /usr/include/alloca.h /usr/include/limits.h
PCA_test.o: /usr/include/bits/posix1_lim.h /usr/include/bits/local_lim.h
PCA_test.o: /usr/include/linux/limits.h /usr/include/bits/posix2_lim.h
PCA_test.o: /astro/users/vanderplas/lib/cfitsio/include/longnam.h
PCA_test.o: /usr/include/math.h /usr/include/bits/huge_val.h
PCA_test.o: /usr/include/bits/mathdef.h /usr/include/bits/mathcalls.h
RLLE.o: ../include/RLLE.h ../include/LLE.h ../include/MatVec.h
RLLE.o: ../include/MatVecExcept.h ../include/MatVecBase.h ../include/blas.h
RLLE.o: ../include/fitstools.h
RLLE.o: /astro/users/vanderplas/lib/cfitsio/include/fitsio.h
RLLE.o: /usr/include/stdio.h /usr/include/features.h /usr/include/sys/cdefs.h
RLLE.o: /usr/include/bits/wordsize.h /usr/include/gnu/stubs.h
RLLE.o: /usr/include/gnu/stubs-64.h /usr/include/bits/types.h
RLLE.o: /usr/include/bits/typesizes.h /usr/include/libio.h
RLLE.o: /usr/include/_G_config.h /usr/include/wchar.h
RLLE.o: /usr/include/bits/wchar.h /usr/include/gconv.h
RLLE.o: /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h
RLLE.o: /usr/include/stdlib.h /usr/include/sys/types.h /usr/include/time.h
RLLE.o: /usr/include/endian.h /usr/include/bits/endian.h
RLLE.o: /usr/include/sys/select.h /usr/include/bits/select.h
RLLE.o: /usr/include/bits/sigset.h /usr/include/bits/time.h
RLLE.o: /usr/include/sys/sysmacros.h /usr/include/bits/pthreadtypes.h
RLLE.o: /usr/include/alloca.h /usr/include/limits.h
RLLE.o: /usr/include/bits/posix1_lim.h /usr/include/bits/local_lim.h
RLLE.o: /usr/include/linux/limits.h /usr/include/bits/posix2_lim.h
RLLE.o: /astro/users/vanderplas/lib/cfitsio/include/longnam.h
RLLE.o: /usr/include/math.h /usr/include/bits/huge_val.h
RLLE.o: /usr/include/bits/mathdef.h /usr/include/bits/mathcalls.h
RLLE.o: ../include/HLLE.h ../include/IRWPCA.h ../include/MatSym.h
RLLE.o: ../include/MatVecDecomp.h ../include/argsort.h
RLLE.o: ../include/DimReduceExcept.h
fits_stat.o: ../include/fitstools.h ../include/MatVec.h
fits_stat.o: ../include/MatVecExcept.h ../include/MatVecBase.h
fits_stat.o: ../include/blas.h
fits_stat.o: /astro/users/vanderplas/lib/cfitsio/include/fitsio.h
fits_stat.o: /usr/include/stdio.h /usr/include/features.h
fits_stat.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
fits_stat.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h
fits_stat.o: /usr/include/bits/types.h /usr/include/bits/typesizes.h
fits_stat.o: /usr/include/libio.h /usr/include/_G_config.h
fits_stat.o: /usr/include/wchar.h /usr/include/bits/wchar.h
fits_stat.o: /usr/include/gconv.h /usr/include/bits/stdio_lim.h
fits_stat.o: /usr/include/bits/sys_errlist.h /usr/include/stdlib.h
fits_stat.o: /usr/include/sys/types.h /usr/include/time.h
fits_stat.o: /usr/include/endian.h /usr/include/bits/endian.h
fits_stat.o: /usr/include/sys/select.h /usr/include/bits/select.h
fits_stat.o: /usr/include/bits/sigset.h /usr/include/bits/time.h
fits_stat.o: /usr/include/sys/sysmacros.h /usr/include/bits/pthreadtypes.h
fits_stat.o: /usr/include/alloca.h /usr/include/limits.h
fits_stat.o: /usr/include/bits/posix1_lim.h /usr/include/bits/local_lim.h
fits_stat.o: /usr/include/linux/limits.h /usr/include/bits/posix2_lim.h
fits_stat.o: /astro/users/vanderplas/lib/cfitsio/include/longnam.h
fitstools.o: ../include/fitstools.h ../include/MatVec.h
fitstools.o: ../include/MatVecExcept.h ../include/MatVecBase.h
fitstools.o: ../include/blas.h
fitstools.o: /astro/users/vanderplas/lib/cfitsio/include/fitsio.h
fitstools.o: /usr/include/stdio.h /usr/include/features.h
fitstools.o: /usr/include/sys/cdefs.h /usr/include/bits/wordsize.h
fitstools.o: /usr/include/gnu/stubs.h /usr/include/gnu/stubs-64.h
fitstools.o: /usr/include/bits/types.h /usr/include/bits/typesizes.h
fitstools.o: /usr/include/libio.h /usr/include/_G_config.h
fitstools.o: /usr/include/wchar.h /usr/include/bits/wchar.h
fitstools.o: /usr/include/gconv.h /usr/include/bits/stdio_lim.h
fitstools.o: /usr/include/bits/sys_errlist.h /usr/include/stdlib.h
fitstools.o: /usr/include/sys/types.h /usr/include/time.h
fitstools.o: /usr/include/endian.h /usr/include/bits/endian.h
fitstools.o: /usr/include/sys/select.h /usr/include/bits/select.h
fitstools.o: /usr/include/bits/sigset.h /usr/include/bits/time.h
fitstools.o: /usr/include/sys/sysmacros.h /usr/include/bits/pthreadtypes.h
fitstools.o: /usr/include/alloca.h /usr/include/limits.h
fitstools.o: /usr/include/bits/posix1_lim.h /usr/include/bits/local_lim.h
fitstools.o: /usr/include/linux/limits.h /usr/include/bits/posix2_lim.h
fitstools.o: /astro/users/vanderplas/lib/cfitsio/include/longnam.h
fitstools.o: ../include/DimReduceExcept.h
