import wx
import os
import time


class PhotoCtrl(wx.App):
    mainLocation = "/home/saurabh/Desktop/r/Bag_to_Depth-master/src/bag2rgbdepth/bag2/"
    locationrgb = mainLocation + "rgb_images1/"
    locationdepth = mainLocation + "depth_images1/"
    rdfiles=[]
    ddfiles=[]
    rd = os.listdir(locationrgb)
    dd = os.listdir(locationdepth)
    print(len(rd))
    for r in range(len(rd)):
        if "jpg" in rd[r]:
            rdfiles.append(rd[r])
    for d in range(len(dd)):
        if "jpg" in dd[d]:
            ddfiles.append(dd[d])
    rdfiles = sorted(sorted(rdfiles), key=len)
    ddfiles = sorted(sorted(ddfiles), key=len)
    imageno = 0

    def __init__(self, redirect=False, filename=None):
        wx.App.__init__(self, redirect, filename)
        self.frame = wx.Frame(None, title='Compare Depth and RGB')

        self.panel = wx.Panel(self.frame)
        self.maxvalrgb=0
        self.maxvald = 0
        self.PhotoMaxSize = 640
        self.createWidgets()
        self.frame.Show()

    def createWidgets(self):
        instructions = 'Compare  RGB and depth Location =' + self.mainLocation
        img = wx.Image(640, 480)
        # create Image panel
        self.imageCtrl = wx.StaticBitmap(self.panel, wx.ID_ANY,
                                         wx.Bitmap(img))
        self.depthCtrl = wx.StaticBitmap(self.panel, wx.ID_ANY,
                                         wx.Bitmap(img))

        self.instructLbl = wx.TextCtrl(self.panel, size=(800, -1))
        self.instructLbl.SetValue(instructions)
        self.instructLbl.SetEditable(False)
        self.instructLbl.SetBackgroundColour((211, 211, 211))

        self.photoTxt = wx.TextCtrl(self.panel, size=(200, -1))
        self.photoTxt.SetEditable(False)
        self.depthTxt = wx.TextCtrl(self.panel, size=(200, -1))
        self.depthTxt.SetEditable(False)
        self.jmpTxt = wx.TextCtrl(self.panel, size=(200, -1))
        self.maxTxtrgb = wx.TextCtrl(self.panel, size=(200, -1))
        self.maxTxtrgb.SetEditable(False)
        self.maxTxtd = wx.TextCtrl(self.panel, size=(200, -1))
        self.maxTxtd.SetEditable(False)
        self.diffTxt = wx.TextCtrl(self.panel, size=(200, -1))
        self.diffTxt.SetEditable(False)

        # create buttons
        nextBtn = wx.Button(self.panel, label='Next')
        prevBtn = wx.Button(self.panel, label='Prev')
        delBtn = wx.Button(self.panel, label='Delete')
        dirBtn = wx.Button(self.panel, label='CDir')
        jmpBtn = wx.Button(self.panel, label='Jump To')
        refreshBtn = wx.Button(self.panel, label='REFRESH')
        # bind buttons
        prevBtn.Bind(wx.EVT_BUTTON, self.onPrev)
        nextBtn.Bind(wx.EVT_BUTTON, self.onNext)
        dirBtn.Bind(wx.EVT_BUTTON, self.onBrowse)
        delBtn.Bind(wx.EVT_BUTTON, self.delete)
        jmpBtn.Bind(wx.EVT_BUTTON, self.jumpto)
        refreshBtn.Bind(wx.EVT_BUTTON, self.refreshall)

        self.slideTimer = wx.Timer(None)
        self.slideTimer.Bind(wx.EVT_TIMER, self.onNextplay)

        # Create Boxes
        self.mainSizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.sizerbtn = wx.BoxSizer(wx.HORIZONTAL)
        self.sizermax = wx.BoxSizer(wx.HORIZONTAL)

        # add the stuff to each other
        self.mainSizer.Add(wx.StaticLine(self.panel, wx.ID_ANY),
                           0, wx.ALL | wx.EXPAND, 5)
        self.mainSizer.Add(self.instructLbl, 0, wx.ALL, 5)
        self.sizermax.Add(self.maxTxtrgb, 0, wx.ALL, 5)
        self.sizermax.Add(self.maxTxtd, 0, wx.ALL, 5)
        self.sizermax.Add(self.diffTxt, 0, wx.ALL, 5)

        self.sizer.Add(self.imageCtrl, 0, wx.ALL, 5)
        self.sizer.Add(self.depthCtrl, 0, wx.ALL, 5)
        self.sizerbtn.Add(prevBtn, 0, wx.ALL, 5)
        self.sizerbtn.Add(nextBtn, 0, wx.ALL, 5)
        self.sizerbtn.Add(delBtn, 0, wx.ALL, 5)
        self.sizerbtn.Add(dirBtn, 0, wx.ALL, 5)
        self.sizerbtn.Add(self.photoTxt, 0, wx.ALL, 5)
        self.sizerbtn.Add(self.depthTxt, 0, wx.ALL, 5)
        self.sizerbtn.Add(jmpBtn, 0, wx.ALL, 5)
        self.sizerbtn.Add(self.jmpTxt, 0, wx.ALL, 5)
        self.sizerbtn.Add(refreshBtn, 0, wx.ALL, 5)

        self.mainSizer.Add(self.sizermax, 0, wx.ALL, 5)
        self.mainSizer.Add(self.sizer, 0, wx.ALL, 5)
        self.mainSizer.Add(self.sizerbtn, 0, wx.ALL, 5)
        self.panel.SetSizer(self.mainSizer)
        self.mainSizer.Fit(self.frame)

        self.panel.Layout()

    def onNext(self, event):
        self.imageno += 1
        self.photoTxt.SetValue(self.rdfiles[self.imageno])
        self.depthTxt.SetValue(self.ddfiles[self.imageno])
        self.maxvalrgb = len(self.rdfiles)
        self.maxvald = len(self.ddfiles)

        self.onView()
    def onNextplay(self):
        self.imageno += 1
        print(self.imageno)
        self.photoTxt.SetValue(self.rdfiles[self.imageno])
        self.depthTxt.SetValue(self.ddfiles[self.imageno])
        self.maxvalrgb = len(self.rdfiles)
        self.maxvald = len(self.ddfiles)
        self.onView()
    def onPrev(self, event):
        self.imageno -= 1
        self.photoTxt.SetValue(self.rdfiles[self.imageno])
        self.depthTxt.SetValue(self.ddfiles[self.imageno])
        self.maxvalrgb = len(self.rdfiles)
        self.maxvald = len(self.ddfiles)
        self.onView()

    def delete(self, event):
        filepath = self.locationrgb + self.photoTxt.GetValue()
        filepath2 = self.locationdepth + self.depthTxt.GetValue()

        os.system("rm " +self.locationrgb+str(self.photoTxt.GetValue())[:-3]+str("*"))
        print(self.locationrgb+str(self.photoTxt.GetValue())[:-3])

        os.system("rm " +self.locationdepth+ str(self.depthTxt.GetValue())[:-3]+str("*"))
        print(self.locationdepth+str(self.depthTxt.GetValue())[:-3])

        rd = os.listdir(self.locationrgb)
        dd = os.listdir(self.locationdepth)
        rdfiles=[]
        ddfiles=[]
        for r in range(len(rd)):
            if "jpg" in rd[r]:
                rdfiles.append(rd[r])
        for d in range(len(dd)):
            if "jpg" in dd[d]:
                ddfiles.append(dd[d])

        self.rdfiles = sorted(sorted(rdfiles), key=len)
        self.ddfiles = sorted(sorted(ddfiles), key=len)
        self.photoTxt.SetValue(self.rdfiles[self.imageno])
        self.depthTxt.SetValue(self.ddfiles[self.imageno])
        self.maxvalrgb = len(self.rdfiles)
        self.maxvald = len(self.ddfiles)

        self.onView()
    def refreshall(self,event):
        rd = os.listdir(self.locationrgb)
        dd = os.listdir(self.locationdepth)
        rdfiles=[]
        ddfiles=[]
        for r in range(len(rd)):
            if "jpg" in rd[r]:
                rdfiles.append(rd[r])
        for d in range(len(dd)):
            if "jpg" in dd[d]:
                ddfiles.append(dd[d])
        self.imageno = 0
        self.rdfiles = sorted(sorted(rdfiles), key=len)
        self.ddfiles = sorted(sorted(ddfiles), key=len)
        self.photoTxt.SetValue(self.rdfiles[self.imageno])
        self.depthTxt.SetValue(self.ddfiles[self.imageno])
        self.maxvalrgb = len(self.rdfiles)
        self.maxvald = len(self.ddfiles)
        self.onView()
    def jumpto(self,event):
        i = int(self.jmpTxt.GetValue())
        print(len(self.rdfiles))
        if i <= len(self.rdfiles):
            self.imageno=i
            self.photoTxt.SetValue(self.rdfiles[self.imageno])
            self.depthTxt.SetValue(self.ddfiles[self.imageno])
            self.onView()
        else:
            print("Too high reduce jump")

    def onBrowse(self, event):
        """
        Browse for file
        """
        dialog = wx.DirDialog(None, "OLD DIR = "+str(self.locationrgb), self.locationrgb,
                           wx.DD_DEFAULT_STYLE | wx.DD_DIR_MUST_EXIST)
        if dialog.ShowModal() == wx.ID_OK:
            self.mainLocation=dialog.GetPath()
        self.locationrgb = self.mainLocation + "/rgb_images1/"
        self.locationdepth = self.mainLocation + "/depth_images1/"

        rd = os.listdir(self.locationrgb)
        dd = os.listdir(self.locationdepth)
        rdfiles=[]
        ddfiles=[]
        for r in range(len(rd)):
            if "jpg" in rd[r]:
                rdfiles.append(rd[r])
        for d in range(len(dd)):
            if "jpg" in dd[d]:
                ddfiles.append(dd[d])

        self.rdfiles = sorted(sorted(rdfiles), key=len)
        self.ddfiles = sorted(sorted(ddfiles), key=len)
        self.imageno = 0
        self.photoTxt.SetValue(self.rdfiles[self.imageno])
        self.depthTxt.SetValue(self.ddfiles[self.imageno])
        dialog.Destroy()
        self.refreshall(None)

    def onView(self):
        filepath = self.locationrgb + self.photoTxt.GetValue()
        filepath2 = self.locationdepth + self.depthTxt.GetValue()
        img = wx.Image(filepath, wx.BITMAP_TYPE_ANY)
        img2 = wx.Image(filepath2, wx.BITMAP_TYPE_ANY)
        # scale the image, preserving the aspect ratio
        W = img.GetWidth()
        H = img.GetHeight()
        if W > H:
            NewW = self.PhotoMaxSize
            NewH = self.PhotoMaxSize * H / W
        else:
            NewH = self.PhotoMaxSize
            NewW = self.PhotoMaxSize * W / H

        img = img.Scale(NewW, NewH)
        img2 = img2.Scale(NewW, NewH)

        self.imageCtrl.SetBitmap(wx.Bitmap(img))
        self.depthCtrl.SetBitmap(wx.Bitmap(img2))
        self.maxTxtrgb.SetValue("Max RGB = "+str(self.maxvalrgb-1))
        self.maxTxtd.SetValue("Max Depth = " + str(self.maxvald-1))
        self.diffTxt.SetValue("Diff = "+str(self.maxvalrgb-self.maxvald))
        self.instructLbl.SetValue('Compare  RGB and depth Location =' + self.mainLocation)
        self.panel.Refresh()


if __name__ == '__main__':
    app = PhotoCtrl()
    app.MainLoop()


