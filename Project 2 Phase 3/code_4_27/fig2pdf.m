function [ iflag ] = fig2pdf( h,fname )
%FIG2PDF save figure as pdf
% h: figure handle
% fname: file name, string
iflag = 0;
% Get original units setting
fig_unit = get(h,'Units');
paper_unit = get(h,'PaperUnits');
% Change units setting to inches
if ~strcmp(fig_unit,'inches')
    set(h,'Units','inches')
end
if ~strcmp(paper_unit,'inches')
    set(h,'PaperUnits','inches')
end
% Set paper position and size to what you have on the screen
fig_position = get(h,'Position');
fig_width = fig_position(3);
fig_height = fig_position(4);
set(h,'PaperPosition',[0 0 fig_width fig_height])
set(h,'PaperSize',[fig_width fig_height])
% Save as pdf
print(h,'-dpdf',fname);
iflag = 1;
% Set units back to original setting
set(h,'Units',fig_unit)
set(h,'PaperUnits',paper_unit)
end

