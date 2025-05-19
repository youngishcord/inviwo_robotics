# Name: dataAnalyzer

import sys

import inviwopy as ivw
from inviwopy.properties import ConstraintBehavior as cb


class dataAnalyzer(ivw.Processor):
    """
    Documentation of template
    """

    def __init__(self, id, name):
        ivw.Processor.__init__(self, id, name)
        
        self.volumeInport = ivw.data.VolumeInport("volume")
        self.addInport(self.volumeInport, owner=False)
        
        self.filePath = ivw.properties.FileProperty("filePath", "filePath")
        self.addProperty(self.filePath)

        self.objectId = ivw.properties.StringProperty("objectId", "objectId")
        self.addProperty(self.objectId)

        self.snapshotBt = ivw.properties.ButtonProperty("snapshotBt", "snapshotBt")
        self.addProperty(self.snapshotBt)
        self.snapshotBt.onChange(self.makeSnapshot)
        
        self.saveBt = ivw.properties.ButtonProperty("saveBt", "saveBt")
        self.addProperty(self.saveBt)
        self.saveBt.onChange(self.saveDataframe)
        
        self.updateplotOnProcess = ivw.properties.BoolProperty("updateOnProcess", "updateOnProcess")
        self.addProperty(self.updateplotOnProcess)
        
    @staticmethod
    def processorInfo():
        return ivw.ProcessorInfo(
            classIdentifier="org.inviwo.dataAnalyzer",
            displayName="dataAnalyzer",
            category="Python",
            codeState=ivw.CodeState.Stable,
            tags=ivw.Tags.PY,
            help=ivw.unindentMd2doc(dataAnalyzer.__doc__)
        )

    def getProcessorInfo(self):
        return dataAnalyzer.processorInfo()

    def initializeResources(self):
        pass    

    def process(self):
        pass 