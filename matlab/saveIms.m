function saveIms(name, testIm, depthIm, depthImDisplay)
    imwrite(testIm,name+"_rgb.png")
    imwrite(depthIm,name+"_d.png")
    imwrite(depthImDisplay,name+"_ddisplay.png")
end