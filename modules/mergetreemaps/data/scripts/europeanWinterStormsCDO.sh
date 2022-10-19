module load CDO/1.9.2-nsc1-intel-2018a-eb

cdo expr,wind="sqrt(u10*u10+v10*v10)" ERA5_WindUV_MLSP_Dec1999_Europe.nc ERA5_Wind_Dec1999_Europe.nc
cdo selvar,msl ERA5_MSLP_Dec1999_Europe.nc
cdo -b F32 mergetime ERA5_MSLP_17-30Nov1999_Europe ERA5_MSLP_Dec1999_Europe.nc ERA5_MSLP_1-14Jan2000_Europe.nc ERA5_MSLP_17Nov1999-14Jan2000_Europe.nc
cdo runmean,193 ERA5_MSLP_17Nov1999-14Jan2000_Europe.nc ERA5_MSLP_RunMean8Days_17Nov1999-14Jan2000_Europe.nc
cdo -selmon,12  ERA5_MSLP_RunMean8Days_17Nov1999-14Jan2000_Europe.nc ERA5_MSLP_RunMean8Days_Dec1999_Europe.nc
cdo -b F32 sub ERA5_MSLP_Dec1999_Europe.nc ERA5_MSLP_RunMean8Days_Dec1999_Europe.nc ERA5_MSLP_Minus_RunMean8Days_Dec1999_Europe.nc