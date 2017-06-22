echo off
@cls

set target_dir=PCIDrvLinux_Backup
set target_file=PCIDrvLinux.zip
rem set hg_option=--clean --modified --added
set hg_option=--modified --added

if exist %target_dir% (rmdir /s /q %target_dir%)
mkdir %target_dir%

echo:
echo "	<<< PCIDrv_Linux >>>    "
echo ------------------------------------------------------------------------------
set hg_repro=PCIDrv_Linux
set hg_cmd=hg status %hg_option% -n -R ../%hg_repro%
mkdir "%target_dir%/%hg_repro%/"
FOR /f "delims=" %%G IN ('%hg_cmd%') DO xcopy /v "..\%hg_repro%\%%G" "%target_dir%\%hg_repro%\%%G"* | find /v "Datei(en) kopiert"



echo:
echo:
echo ------------------------------------------------------------------------------
echo ------------------------------------------------------------------------------
if exist %target_file% (del %target_file%)
"C:\Program Files\7-Zip\7z" a -tzip %target_file%  %target_dir% | find /v "Compressing"
rmdir /s /q %target_dir%
echo:

pause

