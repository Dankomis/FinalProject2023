function [ref,vari]=get_reflectivityData(ptc, x, y, z, R,obj)
filtered_ptc=ptc(sqrt((ptc(:,1)-x).^2+(ptc(:,2)-y).^2+(ptc(:,3)-z).^2)<R,:);
if isempty(filtered_ptc)
    ref=obj.refAvg;
    vari = obj.refVar;
else
    ref=mean(filtered_ptc(:,4));
    % calculate the overall variance
    vari = var(filtered_ptc(:,4));

end