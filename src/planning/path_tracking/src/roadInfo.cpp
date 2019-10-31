/*
max path tracking speed 20kmph
A-B decelaration area 8kmph;
C-D right turn 10kmph;
E-F decelaration area 8kmph;
*/

size_t A = , B = , 
       C = , D = , 
       E = , F = ;

if (nearest_point_index_ > A && nearest_point_index_ <B)
{
    speed_limit_ = 8;
}
else if (nearest_point_index_ <C && nearest_point_index_ >D)
{
    speed _limit_ =10;
    gps_controlCmd_.cmd1.set_turnLight_R = true;
}
else if(nearest_point_index_ >E && nearest_point_index_ < F)
{
    speed_limit_ = 8;
}