function classify_and_update_GUI(handles)

global G_scan

G_scan.ClassificationData=[];

%ORIGINAL COLOUR
% case 1 = 'Grey Metal '; case 2 = 'Shiny Metal'; case 3 = 'Cloth/Wood'; OR RED OR WHITE --- JUST CLOTH or WOOD!!! case 4 = 'Do not know';

%if NOT USING real robot load data
if ~get(handles.useRealRobot_checkbox,'Value')
    display('Inputing data for classification from file');
    load ScanforClassifier-0to-60.mat
    G_scan.IntensityData=IntensityData;
    G_scan.PointData=PointData;
    G_scan.RangeData=RangeData;
end

%check that PointData exists and is valid
if ~isfield(G_scan,'PointData') || ~isfield(G_scan,'IntensityData') || ~isfield(G_scan,'RangeData')
  error('Cant classify since no PointData/IntensityData/RangeData has been saved to the G_scan structure - Take a scan with real robot');    
end


%do classification with data
display('Classifying the LATEST set of data that has been scanned');

try [ClassifiedData] = Block_Classifier(G_scan.PointData, G_scan.IntensityData); 
  try update_ocstatus(ClassifiedData);
        display('....Classification completed successfully');    
  catch  %#ok<CTCH>
      display('Couldnt update voxels'); keyboard; 
  end        
catch %#ok<CTCH>
    display('Couldnt classify');keyboard; 
end
    