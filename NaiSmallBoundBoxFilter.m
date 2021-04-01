%{
% Function:  Naive small bounding-box filter (NSBBF)
% Version:   V1.0.0
% Designer:  Jingneng Fu
% Reference: J. Fu et al., Small bounding-box filter for small target detection, Opt.Eng. 60(3), 033107(2021).
% Date:      2020-05-20
% Input:     ImgIn should be a matrix of double type,and k = 5.0 could be a good chioce.
%}
function ImgOut = NaiSmallBoundBoxFilter(ImgIn,k)
SigLab = zeros(size(ImgIn));
ImgOut = SigLab;
% Sizes of bouding boxes are 5*5-13*13
HalfSize   = 4;
ConGapLow  = 2;     
ConGapHigh = 6;   
%
MagSize = 4*HalfSize;
ImgIn = padarray(ImgIn,[MagSize,MagSize],'symmetric');
[RowSize,ColSize] = size(ImgIn);
% Matrix of threshold
Tm = zeros(RowSize,ColSize);
% Matrix of mean
Mu = zeros(RowSize,ColSize);
%
for row = (MagSize+1):(RowSize-MagSize)
    for col = (MagSize+1):(ColSize-MagSize)
        ImgTemp = ImgIn((row-HalfSize):(row+HalfSize),(col-HalfSize):(col+HalfSize));  
        % Calculate the position of the local maximum gray pixel
        [RowLab,ColLab] = find(ImgTemp == max(ImgTemp(:)));
        %
        RowLab = row - HalfSize -1 + RowLab(1);
        ColLab = col - HalfSize -1 + ColLab(1);
        % Renew Threshold
        if(Tm(RowLab,ColLab)==0) 
            GrayMin = inf;
            m = inf;
            for BaGdGap = ConGapLow:ConGapHigh
                PixBaGd = [ImgIn(RowLab-BaGdGap,(ColLab-BaGdGap):(ColLab+BaGdGap)),...
                           ImgIn(RowLab+BaGdGap,(ColLab-BaGdGap):(ColLab+BaGdGap)),...
                           ImgIn((RowLab-BaGdGap+1):(RowLab+BaGdGap-1),ColLab-BaGdGap)',...
                           ImgIn((RowLab-BaGdGap+1):(RowLab+BaGdGap-1),ColLab+BaGdGap)'];  
                           %
                ave = mean(PixBaGd);
                var = std(PixBaGd);
                % Threshold of NSBBF
                GrayTemp = ave+k*var;
                if(GrayMin>GrayTemp)
                    GrayMin = GrayTemp;
                    % mean
                    m       = ave;
                end
            end       
            Tm(RowLab,ColLab) = GrayMin;
            Mu(RowLab,ColLab) = m;
        end
        %          
        SigLab(row-MagSize,col-MagSize) = ImgIn(row,col)- Tm(RowLab,ColLab);
        ImgOut(row-MagSize,col-MagSize) = ImgIn(row,col)- Mu(RowLab,ColLab);
    end
end
% Result
ImgOut = (SigLab>0).*ImgOut;

