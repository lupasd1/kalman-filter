function tracks = fcn(clusters)

clusters
EstimatesNext = zeros(6, 1);
EstimatesCurrent = zeros(6, 1);
UncertaintyNext = zeros(6, 6, 1);
UncertaintyCurrent = zeros(6, 6, 1);
KalmanGain = zeros(6, 6);

intialEstimate = zeros(6, 1);
initialUncertainty = diag([500 500 500 500 500 500]);

dt = 1;
StDev = 0.2;
MeasureError = 3;

ZeroMat = zeros(3);
A = [1 dt (0.5*dt^2)
     0 1  dt
     0 0  1];
F = [A ZeroMat
     ZeroMat A];

B = [dt^2/4 dt^3/2 dt^2/2
     dt^3/2 dt^2   dt
     dt^2/2 dt     1];

Q = [B ZeroMat
     ZeroMat B] * StDev^2;

R = [MeasureError^2 0
     0 MeasureError^2];

C = [1 0 0];
D = [0 1 0];
E = [0 0 0];
H = [C E
     E C
     E E
     D E
     E D
     E E];


EstimatesNext = F*intialEstimate;
UncertaintyNext = F*initialUncertainty*transpose(F)+Q;


KalmanGain =  (UncertaintyNext*transpose(H)) / (H*UncertaintyNext * transpose(H) + R);
EstimatesCurrent = EstimatesNext + KalmanGain * ([xMeasured(1); yMeasured(1)] - H*EstimatesNext);

identminus = eye(6) - KalmanGain * H;
UncertaintyCurrent = identminus * UncertaintyNext * transpose(identminus) + KalmanGain*R*transpose(KalmanGain);


EstimatesNext = F*EstimatesCurrent;
UncertaintyNext = F * UncertaintyCurrent * transpose(F) + Q;


% placeHolderAttributes.TargetIndex = 1;
% placeHolderAttributes.SNR = 1;
% 
% tracks.NumTracks = 0;
% 
% fakeTrack.TrackID = uint32(1);
% fakeTrack.BranchID = uint32(1);
% fakeTrack.SourceIndex = uint32(1);
% fakeTrack.UpdateTime = 0.1;
% fakeTrack.Age = uint32(1);
% fakeTrack.State = [0; 0; 0; 0; 0; 0];
% fakeTrack.StateCovariance = zeros(6,6);
% fakeTrack.ObjectClassID = 1;
% fakeTrack.TrackLogic = trackLogicType.History;
% fakeTrack.TrackLogicState = [true true true];
% fakeTrack.IsConfirmed = boolean(0);
% fakeTrack.IsCoasted = boolean(false);
% fakeTrack.IsSelfReported = boolean(false);
% fakeTrack.ObjectAttributes = [placeHolderAttributes; placeHolderAttributes];
% 
% for i = 1:length(tracks.Tracks)
%     tracks.Tracks(i) = fakeTrack;
% end
