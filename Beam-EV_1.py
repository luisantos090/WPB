# Save by rabeeshamass on 2021_11_16-09.48.56; build 2020 2019_09_13-18.49.31 163176
from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *
import numpy as np
import math
import matplotlib.pyplot as plt

def PrintToScreen(Message):
    print >> sys.__stdout__, '%s' % Message
    
    
# PATH TO CREATE INCREMENTATION FOLDERS
# Change work directory
os.chdir(r"C:\working files\Eliptical-AI\Parametric_3")

# os.chdir(r"C:\working files\AI_For_SS beam\Luis")

Section = "UB 178x102x19"
d = 177.8
tw = 4.8
bf = 101.2
tf = 7.9

d_H_ratios = np.arange(1.2,1.61,0.1)
d0_H_ratios = np.arange(0.65,0.901,0.05)
R_d0_ratios = np.arange(0.1,0.41,0.05)
w_d0_ratios = np.arange(0.25,0.71,0.1)


BaseDir = os.getcwd()
Foldername = Section
if not os.path.exists(BaseDir+"/"+str(Foldername)):
    os.mkdir(BaseDir+"/"+str(Foldername))
os.chdir(BaseDir+"/"+str(Foldername))


Models = []
   
for hh in d_H_ratios:
    for i in d0_H_ratios:
        # for j in tw_H_ratios:
        for k in R_d0_ratios:
            for l in w_d0_ratios:
                H = round(hh*d,2)
                d0 = round(i*H,2)
                R = round(k*i*H,2)
                w = round(l*i*H,2)
                if w/2.0 > R and 2*R/w > 0.15:
                    Models.append([bf, tf, H, tw, d0, w, R])




# VERY IMPORTANT
session.journalOptions.setValues(replayGeometry=COORDINATE, recoverGeometry=COORDINATE)



def CreateModelWebPostBuckling(variables):

    flange_width = variables[0]
    flange_thickness = variables[1]

    height = variables[2]
    web_thickness = variables[3]

    d0 = variables[4]
    hole_width = variables[5]
    fillet_radius = variables[6]

    #length = variables[4]
    length = hole_width + 2.0*fillet_radius

    web_mesh_size = round(height/200.0,1)
    flange_mesh_size = round(flange_width/20.0,1)
    
    imperfections = height/500.0

    dist1 = (height - d0)/2.0
    dist2 = d0/2.0 - fillet_radius
    
    load = 3.0*web_thickness*355.0/math.sqrt(3)


    # Create New Forder
    NewFolder = BaseDir+"/"+str(Foldername)+"/"+str(flange_width)+"_"+str(flange_thickness)+"_"+str(height)+"_"+str(web_thickness)+"_"+str(d0)+"_"+str(hole_width)+"_"+str(fillet_radius)
    PrintToScreen(str(flange_width)+"_"+str(flange_thickness)+"_"+str(height)+"_"+str(web_thickness)+"_"+str(d0)+"_"+str(hole_width)+"_"+str(fillet_radius))
    os.mkdir(NewFolder)
    os.chdir(NewFolder)

    #Create New Model
    Mdb()


    #Sketch main I-beam geometry
    mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=200.0)
    mdb.models['Model-1'].sketches['__profile__'].Line(point1=(0.0, height/2.0), point2=
        (0.0, -height/2.0))
    mdb.models['Model-1'].sketches['__profile__'].VerticalConstraint(addUndoState=
        False, entity=mdb.models['Model-1'].sketches['__profile__'].geometry[2])
    mdb.models['Model-1'].sketches['__profile__'].Line(point1=(0.0, height/2.0), point2=
        (flange_width/2.0, height/2.0))
    mdb.models['Model-1'].sketches['__profile__'].HorizontalConstraint(
        addUndoState=False, entity=
        mdb.models['Model-1'].sketches['__profile__'].geometry[3])
    mdb.models['Model-1'].sketches['__profile__'].Line(point1=(0.0, height/2.0), point2=
        (-flange_width/2.0, height/2.0))
    mdb.models['Model-1'].sketches['__profile__'].HorizontalConstraint(
        addUndoState=False, entity=
        mdb.models['Model-1'].sketches['__profile__'].geometry[4])
    mdb.models['Model-1'].sketches['__profile__'].Line(point1=(0.0, -height/2.0), 
        point2=(flange_width/2.0, -height/2.0))
    mdb.models['Model-1'].sketches['__profile__'].HorizontalConstraint(
        addUndoState=False, entity=
        mdb.models['Model-1'].sketches['__profile__'].geometry[5])
    mdb.models['Model-1'].sketches['__profile__'].Line(point1=(0.0, -height/2.0), 
        point2=(-flange_width/2.0, -height/2.0))
    mdb.models['Model-1'].sketches['__profile__'].HorizontalConstraint(
        addUndoState=False, entity=
        mdb.models['Model-1'].sketches['__profile__'].geometry[6])
    mdb.models['Model-1'].Part(dimensionality=THREE_D, name='Part-1', type=
        DEFORMABLE_BODY)
    mdb.models['Model-1'].parts['Part-1'].BaseShellExtrude(depth=length, sketch=
        mdb.models['Model-1'].sketches['__profile__'])
    del mdb.models['Model-1'].sketches['__profile__']

    #Geometry of the cut
    mdb.models['Model-1'].ConstrainedSketch(gridSpacing=18.02, name='__profile__', 
        sheetSize=721.11, transform=
        mdb.models['Model-1'].parts['Part-1'].MakeSketchTransform(
        sketchPlane=mdb.models['Model-1'].parts['Part-1'].faces[1], 
        sketchPlaneSide=SIDE1, 
        sketchUpEdge=mdb.models['Model-1'].parts['Part-1'].edges[4], 
        sketchOrientation=RIGHT, origin=(0.0, 0.0, length/2.0)))
    mdb.models['Model-1'].parts['Part-1'].projectReferencesOntoSketch(filter=
        COPLANAR_EDGES, sketch=mdb.models['Model-1'].sketches['__profile__'])
    mdb.models['Model-1'].sketches['__profile__'].ArcByCenterEnds(center=(length/2.0, 
        dist2), direction=COUNTERCLOCKWISE, point1=(length/2.0, d0/2.0), point2=(length/2.0-fillet_radius, 
        dist2))
    mdb.models['Model-1'].sketches['__profile__'].Line(point1=(length/2.0-fillet_radius, dist2), 
        point2=(length/2.0-hole_width/2.0, 0.0))
    mdb.models['Model-1'].sketches['__profile__'].ConstructionLine(point1=(length/2.0, 
        0.0), point2=(-length/2.0, 0.0))
    mdb.models['Model-1'].sketches['__profile__'].HorizontalConstraint(
        addUndoState=False, entity=
        mdb.models['Model-1'].sketches['__profile__'].geometry[12])
    mdb.models['Model-1'].sketches['__profile__'].ConstructionLine(point1=(0.0, 
        height), point2=(0.0, -height))
    mdb.models['Model-1'].sketches['__profile__'].VerticalConstraint(addUndoState=
        False, entity=mdb.models['Model-1'].sketches['__profile__'].geometry[13])
    mdb.models['Model-1'].sketches['__profile__'].copyMirror(mirrorLine=
        mdb.models['Model-1'].sketches['__profile__'].geometry[12], objectList=(
        mdb.models['Model-1'].sketches['__profile__'].geometry[10], 
        mdb.models['Model-1'].sketches['__profile__'].geometry[11]))
    mdb.models['Model-1'].sketches['__profile__'].Line(point1=(length/2.0, d0/2.0), 
        point2=(length/2.0, -d0/2.0))
    mdb.models['Model-1'].sketches['__profile__'].VerticalConstraint(addUndoState=
        False, entity=mdb.models['Model-1'].sketches['__profile__'].geometry[16])
    mdb.models['Model-1'].sketches['__profile__'].copyMirror(mirrorLine=
        mdb.models['Model-1'].sketches['__profile__'].geometry[13], objectList=(
        mdb.models['Model-1'].sketches['__profile__'].geometry[10], 
        mdb.models['Model-1'].sketches['__profile__'].geometry[11], 
        mdb.models['Model-1'].sketches['__profile__'].geometry[15], 
        mdb.models['Model-1'].sketches['__profile__'].geometry[14]))
    mdb.models['Model-1'].sketches['__profile__'].Line(point1=(-length/2.0, d0/2.0), 
        point2=(-length/2.0, -d0/2.0))
    mdb.models['Model-1'].sketches['__profile__'].VerticalConstraint(addUndoState=
        False, entity=mdb.models['Model-1'].sketches['__profile__'].geometry[21])
    mdb.models['Model-1'].parts['Part-1'].CutExtrude(flipExtrudeDirection=OFF, 
        sketch=mdb.models['Model-1'].sketches['__profile__'], sketchOrientation=
        RIGHT, sketchPlane=mdb.models['Model-1'].parts['Part-1'].faces[1], 
        sketchPlaneSide=SIDE1, sketchUpEdge=
        mdb.models['Model-1'].parts['Part-1'].edges[4])
    del mdb.models['Model-1'].sketches['__profile__']

    #Material Properties
    mdb.models['Model-1'].Material(name='S355')
    mdb.models['Model-1'].materials['S355'].Elastic(table=((200000.0, 0.3), ))
    mdb.models['Model-1'].materials['S355'].Plastic(table=((355.63, 0.0), (361.19, 
        0.0155), (571.0, 0.1501)))
        
    #Section Definition
    mdb.models['Model-1'].HomogeneousShellSection(idealization=NO_IDEALIZATION, 
        integrationRule=SIMPSON, material='S355', name='FlangeSec', 
        nodalThicknessField='', numIntPts=5, poissonDefinition=DEFAULT, 
        preIntegrate=OFF, temperature=GRADIENT, thickness=flange_thickness, thicknessField='', 
        thicknessModulus=None, thicknessType=UNIFORM, useDensity=OFF)
    mdb.models['Model-1'].HomogeneousShellSection(idealization=NO_IDEALIZATION, 
        integrationRule=SIMPSON, material='S355', name='WebSec', 
        nodalThicknessField='', numIntPts=5, poissonDefinition=DEFAULT, 
        preIntegrate=OFF, temperature=GRADIENT, thickness=web_thickness, thicknessField='', 
        thicknessModulus=None, thicknessType=UNIFORM, useDensity=OFF)
        
    # Assign Sections  
    mdb.models['Model-1'].parts['Part-1'].Set(faces=
        mdb.models['Model-1'].parts['Part-1'].faces.getSequenceFromMask(('[#2 ]', 
        ), ), name='Set-1')
    mdb.models['Model-1'].parts['Part-1'].SectionAssignment(offset=0.0, 
        offsetField='', offsetType=MIDDLE_SURFACE, region=
        mdb.models['Model-1'].parts['Part-1'].sets['Set-1'], sectionName='WebSec', 
        thicknessAssignment=FROM_SECTION)
    mdb.models['Model-1'].parts['Part-1'].Set(faces=
        mdb.models['Model-1'].parts['Part-1'].faces.getSequenceFromMask(('[#1d ]', 
        ), ), name='Set-2')
    mdb.models['Model-1'].parts['Part-1'].SectionAssignment(offset=0.0, 
        offsetField='', offsetType=MIDDLE_SURFACE, region=
        mdb.models['Model-1'].parts['Part-1'].sets['Set-2'], sectionName=
        'FlangeSec', thicknessAssignment=FROM_SECTION)
        
    # Assembly
    mdb.models['Model-1'].rootAssembly.DatumCsysByDefault(CARTESIAN)
    mdb.models['Model-1'].rootAssembly.Instance(dependent=OFF, name='Part-1-1', 
        part=mdb.models['Model-1'].parts['Part-1'])
    mdb.models['Model-1'].rootAssembly.Set(edges=
        mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].edges.getSequenceFromMask(
        ('[#920001 ]', ), ), name='Flange -LHS')
    mdb.models['Model-1'].rootAssembly.Set(edges=
        mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].edges.getSequenceFromMask(
        ('[#1280004 ]', ), ), name='Flange-RHS')
    mdb.models['Model-1'].rootAssembly.Set(edges=
        mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].edges.getSequenceFromMask(
        ('[#840 ]', ), ), name='Web-LHS')
    mdb.models['Model-1'].rootAssembly.Set(edges=
        mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].edges.getSequenceFromMask(
        ('[#2020 ]', ), ), name='Web-RHS')
    mdb.models['Model-1'].rootAssembly.Surface(name='Web-RHS-Load', side1Edges=
        mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].edges.getSequenceFromMask(
        ('[#2020 ]', ), ))
    mdb.models['Model-1'].rootAssembly.Set(faces=
        mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.getSequenceFromMask(
        ('[#2 ]', ), ), name='Web-mesh')
    mdb.models['Model-1'].rootAssembly.Set(edges=
        mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].edges.getSequenceFromMask(
        ('[#1ba0005 ]', ), ), name='Flange-mesh')
        
    # Buckling Analysis Step
    mdb.models['Model-1'].BuckleStep(maxIterations=100, name='Step-1', numEigen=5, 
        previous='Initial', vectors=10)
    
    # Boundary Conditions
    mdb.models['Model-1'].DisplacementBC(amplitude=UNSET, createStepName='Initial', 
        distributionType=UNIFORM, fieldName='', localCsys=None, name='Flange-LHS', 
        region=mdb.models['Model-1'].rootAssembly.sets['Flange -LHS'], u1=SET, u2=
        SET, u3=SET, ur1=UNSET, ur2=UNSET, ur3=UNSET)
    mdb.models['Model-1'].DisplacementBC(amplitude=UNSET, createStepName='Initial', 
        distributionType=UNIFORM, fieldName='', localCsys=None, name='Flange-RHS', 
        region=mdb.models['Model-1'].rootAssembly.sets['Flange-RHS'], u1=SET, u2=
        UNSET, u3=UNSET, ur1=UNSET, ur2=SET, ur3=SET)
    mdb.models['Model-1'].DisplacementBC(amplitude=UNSET, createStepName='Initial', 
        distributionType=UNIFORM, fieldName='', localCsys=None, name='Web-LHS', 
        region=mdb.models['Model-1'].rootAssembly.sets['Web-LHS'], u1=SET, u2=SET, 
        u3=SET, ur1=UNSET, ur2=UNSET, ur3=UNSET)
    mdb.models['Model-1'].DisplacementBC(amplitude=UNSET, createStepName='Initial', 
        distributionType=UNIFORM, fieldName='', localCsys=None, name='Web-RHS', 
        region=mdb.models['Model-1'].rootAssembly.sets['Web-RHS'], u1=SET, u2=UNSET
        , u3=UNSET, ur1=SET, ur2=SET, ur3=UNSET)
        
    # Loads
    mdb.models['Model-1'].ShellEdgeLoad(createStepName='Step-1', distributionType=
        UNIFORM, field='', localCsys=None, magnitude=1.0, name='Load-1', region=
        mdb.models['Model-1'].rootAssembly.surfaces['Web-RHS-Load'], traction=
        TRANSVERSE)
    mdb.models['Model-1'].loads['Load-1'].setValues(traction=SHEAR)
    
    # # Web as a set for History output of out-of-plane displacement
    # y_coord = (d0 - 2.0*fillet_radius)/4.0
    # z_coord = (hole_width/2.0 - fillet_radius)/2.0 + fillet_radius
    # edges1 = mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].edges.findAt(((0.0, -y_coord, z_coord ), ), ((0.0, y_coord, z_coord), ))
    # mdb.models['Model-1'].rootAssembly.Set(edges=edges1, name='Web')


    #Mesh sizes
    mdb.models['Model-1'].rootAssembly.seedEdgeBySize(constraint=FINER, 
        deviationFactor=0.1, edges=
        mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].edges.getSequenceFromMask(
        ('[#1fff2 ]', ), ), size=web_mesh_size)
    mdb.models['Model-1'].rootAssembly.seedEdgeBySize(constraint=FINER, 
        deviationFactor=0.1, edges=
        mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].edges.getSequenceFromMask(
        ('[#1ba0005 ]', ), ), size=flange_mesh_size)
    mdb.models['Model-1'].rootAssembly.generateMesh(regions=(
        mdb.models['Model-1'].rootAssembly.instances['Part-1-1'], ))
    mdb.models['Model-1'].rootAssembly.setElementType(elemTypes=(ElemType(
        elemCode=S4, elemLibrary=STANDARD, secondOrderAccuracy=OFF), ElemType(
        elemCode=S3, elemLibrary=STANDARD)), regions=(
        mdb.models['Model-1'].rootAssembly.instances['Part-1-1'].faces.getSequenceFromMask(
        ('[#1f ]', ), ), ))
        
        
    # To capture buckling modes
    mdb.models['Model-1'].keywordBlock.synchVersions(storeNodesAndElements=False)
    keysword_block = mdb.models['Model-1'].keywordBlock.sieBlocks
    for i in range(len(keysword_block)):
        if keysword_block[i] == '*Output, field, variable=PRESELECT':
            index = i
    mdb.models['Model-1'].keywordBlock.replace(index, '\n*Output, field, variable=PRESELECT\n*Node file\nU')



    mdb.Job(atTime=None, contactPrint=OFF, description='', echoPrint=OFF, 
        explicitPrecision=SINGLE, getMemoryFromAnalysis=True, historyPrint=OFF, 
        memory=90, memoryUnits=PERCENTAGE, model='Model-1', modelPrint=OFF, 
        multiprocessingMode=DEFAULT, name='B-EV', nodalOutputPrecision=SINGLE, 
        numCpus=1, numGPUs=0, queue=None, resultsFormat=ODB, scratch='', type=
        ANALYSIS, userSubroutine='', waitHours=0, waitMinutes=0)
        
        
    mdb.jobs['B-EV'].submit(consistencyChecking=OFF)
    
    
    # TO WAIT FOR JOB COMPLETION
    
    mdb.jobs['B-EV'].waitForCompletion()
    print("Buckling analysis finished running")
    
    odb = session.openOdb(NewFolder+'/'+'B-EV.odb')
    if float(odb.steps['Step-1'].frames[1].description[28:]) > 0:
        Mode = 1
    else: 
        Mode = 2
    
    odb.close()
    
    
    mdb.models['Model-1'].StaticStep(initialInc=0.05, maintainAttributes=True, 
        maxInc=0.05, maxNumInc=1000, minInc=0.001, name='Step-1', nlgeom=ON, previous='Initial')
    mdb.models['Model-1'].loads['Load-1'].setValues(magnitude=load)
    mdb.models['Model-1'].fieldOutputRequests['F-Output-1'].setValues(variables=('S', 'U', 'RF', 'CF', 'TF'))
    
    # # History Output
    # regionDef=mdb.models['Model-1'].rootAssembly.sets['Web']
    # mdb.models['Model-1'].HistoryOutputRequest(name='H-Output-2', 
        # createStepName='Step-1', variables=('U1', ), frequency=LAST_INCREMENT, 
        # region=regionDef, sectionPoints=DEFAULT, rebar=EXCLUDE)
    
    mdb.models['Model-1'].keywordBlock.synchVersions(storeNodesAndElements=False)
    keysword_block = mdb.models['Model-1'].keywordBlock.sieBlocks
    for i in range(len(keysword_block)):
        if keysword_block[i] == '** ----------------------------------------------------------------\n** \n** STEP: Step-1\n** ':
            index = i
    mdb.models['Model-1'].keywordBlock.replace(index, 
        '\n** ----------------------------------------------------------------\n** \n*IMPERFECTION, FILE=B-EV, STEP=1\n%s,%f\n** STEP: Step-1\n**' % (Mode, imperfections))
        
    mdb.Job(atTime=None, contactPrint=OFF, description='', echoPrint=OFF, 
        explicitPrecision=SINGLE, getMemoryFromAnalysis=True, historyPrint=OFF, 
        memory=90, memoryUnits=PERCENTAGE, model='Model-1', modelPrint=OFF, 
        multiprocessingMode=DEFAULT, name='B-NL', nodalOutputPrecision=SINGLE, 
        numCpus=1, numGPUs=0, queue=None, resultsFormat=ODB, scratch='', type=
        ANALYSIS, userSubroutine='', waitHours=0, waitMinutes=0)
        
    # SUBMIT THE JOB

    mdb.jobs['B-NL'].submit(consistencyChecking=OFF)
    
    
    # TO WAIT FOR JOB COMPLETION
    
    mdb.jobs['B-NL'].waitForCompletion()
    print("Static Analysis finished running")


def PostProcessingWebPostBuckling(variables):
    
    flange_width = variables[0]
    flange_thickness = variables[1]

    height = variables[2]
    web_thickness = variables[3]

    # length = variables[4]

    d0 = variables[4]
    hole_width = variables[5]
    fillet_radius = variables[6]

    web_mesh_size = 3.0
    flange_mesh_size = 8.0

    dist1 = (height - d0)/2.0
    dist2 = d0/2.0 - fillet_radius
    
    length = hole_width + 2*fillet_radius
    
    # Create New Forder
    NewFolder = BaseDir+"/"+str(Foldername)+"/"+str(flange_width)+"_"+str(flange_thickness)+"_"+str(height)+"_"+str(web_thickness)+"_"+str(d0)+"_"+str(hole_width)+"_"+str(fillet_radius)
    # os.chdir(NewFolder)
    
    odb = session.openOdb(NewFolder+'/'+'B-EV.odb')
    if float(odb.steps['Step-1'].frames[1].description[28:]) > 0:
        Mode = float(odb.steps['Step-1'].frames[1].description[28:])
    else: 
        Mode = float(odb.steps['Step-1'].frames[2].description[28:])
    
    odb.close()
    
    odb = session.openOdb(NewFolder+'/'+'B-NL.odb')
    
    NrOfSteps = len(odb.steps['Step-1'].frames)

    # for i in range(NrOfSteps):
        # displacement = odb.steps['Step-1'].frames[i].fieldOutputs['U'].values[23].data[1]
        # force = odb.xyDataListFromField(odb=odb, outputPosition=NODAL, variable=(('RF', 
            # NODAL, ((COMPONENT, 'RF2'), )), ), nodeSets=("Point load", ))
        # Displacements_midspan.append(displacement)
        # Applied_forces.append(force)
        
    
    ############################## SAVE DATA IN EXCEL FILE ###########################
    
    Displacements = []
    Applied_forces = []
    
    Disps = odb.steps['Step-1'].frames[-1].fieldOutputs['U'].getSubset(position = NODAL).bulkDataBlocks[0].data
    MaxDisps = np.max(Disps,axis=0)
    MinDisps = np.min(Disps,axis=0)
        
    opFile = NewFolder+'/'+'DatainExcel.csv'
    
    try:
        opFileU = open(opFile,'w')
        opFileU.write("%10s,%10s,%10s,%10s,%10s,%10f,%10s,%10f,%10s,%10f,%10s,%10f,%10s,%10f,%10s,%10f,%10s,%10f\n" % 
		('Time', 'U2', 'applied_force_Y', 'applied_force_Z', 'Mode 1', Mode, 
		'U1_max', MaxDisps[0], 'U1_min', MinDisps[0], 'U2_max', MaxDisps[1], 
		'U2_min', MinDisps[1], 'U3_max', MaxDisps[2] , 'U3_min', MinDisps[2]))

    except IOError:
        PrintToScreen('cannot open opFILE')
        exit(0)

    for i in range(NrOfSteps):
        displacement = odb.steps['Step-1'].frames[i].fieldOutputs['U'].values[19].data[1]
        time = odb.steps['Step-1'].frames[i].frameValue
        applied_force_Y = np.sum(odb.steps['Step-1'].frames[i].fieldOutputs['TF'].values).data[1]
        applied_force_Z = np.sum(odb.steps['Step-1'].frames[i].fieldOutputs['TF'].values).data[2]
        Displacements.append(-displacement)
        Applied_forces.append(applied_force_Y)
        opFileU.write("%10f,%10f,%10f,%10f\n" % (time,displacement, applied_force_Y, applied_force_Z))
        # opFileU.write("%10f,%10f,%10f\n" % (force, displacement, Current_Stiffness))
        
    opFileU.close()
    

    o1 = session.openOdb(name=NewFolder+'/'+'B-NL.odb')
    session.viewports['Viewport: 1'].setValues(displayedObject=o1)
    session.viewports['Viewport: 1'].odbDisplay.display.setValues(plotState=(
        CONTOURS_ON_DEF, ))
    session.viewports['Viewport: 1'].odbDisplay.commonOptions.setValues(deformationScaling=UNIFORM,
        uniformScaleFactor=5)
    
    session.viewports['Viewport: 1'].view.setValues(nearPlane=393.599, 
        farPlane=861.855, width=416.117, height=188.227, viewOffsetX=5.99062, 
        viewOffsetY=1.38402)
    session.viewports['Viewport: 1'].view.fitView()
    session.printToFile(fileName='3D_Mises', format=PNG, canvasObjects=(
        session.viewports['Viewport: 1'], ))
        
    session.viewports['Viewport: 1'].odbDisplay.setPrimaryVariable(
        variableLabel='U', outputPosition=NODAL, refinement=(COMPONENT, 'U1'), )
    session.printToFile(fileName='3D_U1', format=PNG, canvasObjects=(
        session.viewports['Viewport: 1'], ))
          
   
    session.viewports['Viewport: 1'].view.setValues(cameraPosition=(-1.90735e-06, 
        -0.00192261, 700.064), cameraUpVector=(0, 1, 0))
    session.viewports['Viewport: 1'].view.fitView()
    session.printToFile(fileName='XY_U1', format=PNG, canvasObjects=(
        session.viewports['Viewport: 1'], ))
   

    session.viewports['Viewport: 1'].view.setValues(cameraPosition=(569.981, 
        -3.73578, 76.3356))
    session.viewports['Viewport: 1'].view.fitView()
    session.printToFile(fileName='YZ_U1', format=PNG, canvasObjects=(
        session.viewports['Viewport: 1'], ))
    
    session.viewports['Viewport: 1'].odbDisplay.setPrimaryVariable(
        variableLabel='S', outputPosition=INTEGRATION_POINT, refinement=(
        INVARIANT, 'Mises'), )
    session.viewports['Viewport: 1'].view.fitView()
    session.printToFile(fileName='YZ_Mises', format=PNG, canvasObjects=(
        session.viewports['Viewport: 1'], ))
    
       
    fig, ax = plt.subplots()
    ax.plot(Displacements, Applied_forces)

    ax.set(xlabel='Displacements (mm)', ylabel='Load (N/mm)',
           title='Force Displacement Curve')
    ax.grid()

    fig.savefig("Force_Displacement.png")
    plt.close(fig)

    
    odb.close()
       
    PrintToScreen("Post-processing has finished")


# flange_width, flange_thickness, height, web_thickness, d0, hole_width, fillet_radius


for variables in Models:
    flange_width = variables[0]
    flange_thickness = variables[1]
    height = variables[2]
    web_thickness = variables[3]
    d0 = variables[4]
    hole_width = variables[5]
    fillet_radius = variables[6]
    
    NameFolder = BaseDir+"/"+str(Foldername)+"/"+str(flange_width)+"_"+str(flange_thickness)+"_"+str(height)+"_"+str(web_thickness)+"_"+str(d0)+"_"+str(hole_width)+"_"+str(fillet_radius)
    if os.path.exists("%s" % NameFolder):
        continue
        # Max_load, Stiffness_ratio = PostProcessingBeamRabee(variables)
    else:
        CreateModelWebPostBuckling(variables)
        PostProcessingWebPostBuckling(variables)
        for fname in os.listdir(NameFolder):
            if fname.startswith("B-EV") or fname.startswith("B-NL"):
                os.remove(os.path.join(NameFolder, fname))
        # Max_load, Stiffness_ratio = PostProcessingBeamRabee(variables)
    # TrainingData[-1].append(Max_load)
    # TrainingData[-1].append(Stiffness_ratio)
    os.chdir(BaseDir+"/"+str(Foldername))

