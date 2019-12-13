# Lung-GUI    
Developed a computer-aided image processing scheme with a graphic user interface (GUI) model to segment and quantify lung tumors and emphysema using lung CT images.   
**Background**: Lung cancer and chronic obstructive pulmonary disease (COPD) are two major lung diseases with high mortality rates. COPD is also an important risk factor of developing lung cancer. Emphysema is one of major components in COPD. In clinical practice, CT images are widely used to detect and diagnose lung cancer and COPD. However, reading and interpreting lung CT images by radiologists is difficult and has large inter-reader variability. Thus, to more accurately predict or assess prognosis of lung cancer or COPD, it is important to segment and quantify lung tumor and emphysema (volume and other image features).      
**Images/Dataset**: One lung CT case acquired from a public database of lung cancer (LIDC-IDRI-0003)          https://imaging.cancer.gov/informatics/lidc_idri.htm) is provided to develop this scheme and GUI. The total number of CT slice is 140 in this case. The size of each voxel (x, y, z) can be identified from DICOM header of the images.  
## User Manual
This manual helps to understand the LungGUI that can be used to segment the lungs from the Dicom images. In addition, it highlights emphysema using batch processing. LungGUI is also can be used to segment the tumor automatically and manually.
The GUI can be divided into three panels:
1.	Control Panel
2.	Image Panel
3.	Feature Panel   

**Control Panel**: It consists of four push buttons as shown in figure 1.    
![Capturecontrolpanel](https://user-images.githubusercontent.com/44653871/70819925-08a01000-1d9d-11ea-9818-3fb6ca04c0b0.PNG)

![CaptureLoadImage](https://user-images.githubusercontent.com/44653871/70820466-5bc69280-1d9e-11ea-86dd-d6712665008a.PNG)

