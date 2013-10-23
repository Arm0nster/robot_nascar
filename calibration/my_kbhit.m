function my_kbhit( src, event_data )
%=========================================================================
% function my_kbhit( src, event_data )
% - Replicates the functionality of kbhit in "C".  This must be used in
%   conjunction with a figure object. 
% - You need to declare a global variable "kbhit" in the main program
% - You also need to associate the function with keypresses to the active
%   figure, i.e.,  figure('KeyPressFcn', @my_kbhit);
% - The arguments are passed automatically with the key press
%=========================================================================
global kbhit;
kbhit = true;
