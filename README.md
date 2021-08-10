# Post-Filtering

# Compile&Build
1. Download the code  
2. Open the Bundle_adjustment.sln with VS2019  
3. Build&Run (".exe" file is located in /x64/Release/Disp_Filtering.exe )

# Usage
Disp_Filtering.exe filter_prj.xml  
For detailed instruction, pls refer to manual.docx
## Input
filter_prj.xml as shown below:  
![Y`~P{1`7}7 MJ~_`L)K1Q5R](https://user-images.githubusercontent.com/32317924/128939549-789603df-70ed-4b8b-ab0b-aceedd84aadb.png)

It requires:  
1. dsm file path  
![S2Q)GTI DF98 FZC(ER~VFI](https://user-images.githubusercontent.com/32317924/128939676-58375de2-970d-4bec-be80-97f2d92511a5.png)
2. ortho_rgb file path  
![@0~ITUK6JZO$4@4 3{$6GVI](https://user-images.githubusercontent.com/32317924/128939745-8c089a59-0158-4690-9948-e24abc9c3b6a.png)
3. output dir path  

## Output
filterd dsm  
![~W}EW8YIE}J)O~ 7D}IJP_E](https://user-images.githubusercontent.com/32317924/128939855-022ac521-7bea-4abd-968e-68a7008bda89.png)

# This repo is not maintained anymore.
You can still use the code for research purpose, but we are not responsible for any updates.

# Reference
    @article{huang2020post,
      title={Post-filtering with surface orientation constraints for stereo dense image matching},
      author={Huang, Xu and Qin, Rongjun},
      journal={The Photogrammetric Record},
      volume={35},
      number={171},
      pages={375--401},
      year={2020},
      publisher={Wiley Online Library}
    }
