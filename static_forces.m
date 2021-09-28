%Calculating forces on control arms
%Start by defining positions inner
up_fore_in = [-6.224188, 29.842106, 8.740629]./12;   
up_aft_in = [3.998189, 29.842116, 8.740629]./12;     
low_fore_in = [-6.224185, 27.34047, 3.240656]./12;     
low_aft_in = [3.99004, 27.342056, 3.2040656]./12;     
pull_rod_in = [-2.065585, 28.680655, 2.443435]./12;  
tie_rod_in = [2.934415, 27.09608, 5.041408]./12;
%Start by defining positions outer
up_fore_out = [0.225802, 41.176009, 12.024274]./12;   
up_aft_out = [0.225802, 41.176009, 12.024274]./12;     
low_fore_out = [-0.224199, 41.344070, 4.900004]./12;     
low_aft_out = [-0.224199, 41.344070, 4.900004]./12;     
pull_rod_out = [-0.030804, 39.226056, 10.938774]./12;  
tie_rod_out = [2.623612, 40.748339, 7.372449]./12;
%applied forces from tire
f_x = [2.805615601691011, 0, 0];
f_y = [0, 586.4411034361542, 0];
f_z = [0, 0, 250];
%tire_contact_patch positions
x_tire_contact_patch = 0.194602/12;
y_tire_contact_patch = 43.589345/12;
z_tire_contact_patch = 0.472324/12;
r = [x_tire_contact_patch, y_tire_contact_patch, z_tire_contact_patch]; 
%applied moments
moment_applied_x = cross(r, f_x);
moment_applied_y = cross(r, f_y);
moment_applied_z = cross(r, f_z);
moment_applied=[(moment_applied_x(1)+moment_applied_y(1)+moment_applied_z(1)), (moment_applied_x(2)+moment_applied_y(2)+moment_applied_z(2)),(moment_applied_x(3)+moment_applied_y(3)+moment_applied_z(3))];
%unit vectors
up_fore = (up_fore_out - up_fore_in)/norm(up_fore_out - up_fore_in);
up_aft = (up_aft_out - up_aft_in)/norm(up_aft_out - up_aft_in);
low_fore = (low_fore_out - low_fore_in)/norm(low_fore_out - low_fore_in);
low_aft = (low_aft_out - low_aft_in)/norm(low_aft_out - low_aft_in);
pull_rod = (pull_rod_out - pull_rod_in)/norm(pull_rod_out - pull_rod_in);
tie_rod = (tie_rod_out - tie_rod_in)/norm(tie_rod_out - tie_rod_in);
%moment center
moment_center = [0, 0, 0];
%moment unit vector
moment_unit_vector_up_fore = [(moment_center(1) - up_fore(1)),(moment_center(2) - up_fore(2)),(moment_center(3) - up_fore(3))];
moment_unit_vector_up_aft = [(moment_center(1) - up_aft(1)),(moment_center(2) - up_aft(2)),(moment_center(3) - up_aft(3))];
moment_unit_vector_low_fore = [(moment_center(1) - low_fore(1)),(moment_center(2) - low_fore(2)),(moment_center(3) - low_fore(3))];
moment_unit_vector_low_aft = [(moment_center(1) - low_aft(1)),(moment_center(2) - low_aft(2)),(moment_center(3) - low_aft(3))];
moment_unit_vector_pull_rod = [(moment_center(1) - pull_rod(1)),(moment_center(2) - pull_rod(2)),(moment_center(3) - pull_rod(3))];
moment_unit_vector_tie_rod = [(moment_center(1) - tie_rod(1)),(moment_center(2) - tie_rod(2)),(moment_center(3) - tie_rod(3))];
sum_f_x = [up_fore(1), up_aft(1), low_fore(1), low_aft(1), pull_rod(1), tie_rod(1)];
sum_f_y = [up_fore(2), up_aft(2), low_fore(2), low_aft(2), pull_rod(2), tie_rod(2)];
sum_f_z = [up_fore(3), up_aft(3), low_fore(3), low_aft(3), pull_rod(3), tie_rod(3)];
moment_cross_up_fore = cross(moment_unit_vector_up_fore, up_fore_in);
moment_cross_up_aft = cross(moment_unit_vector_up_aft, up_aft_in);
moment_cross_low_fore = cross(moment_unit_vector_low_fore, low_fore_in);
moment_cross_low_aft = cross(moment_unit_vector_low_aft, low_aft_in);
moment_cross_pull_rod = cross(moment_unit_vector_pull_rod, pull_rod_in);
moment_cross_tie_rod = cross(moment_unit_vector_tie_rod, tie_rod_in);
moment_x = [moment_cross_up_fore(1), moment_cross_up_aft(1),moment_cross_low_fore(1), ...
    moment_cross_low_aft(1), moment_cross_pull_rod(1), moment_cross_tie_rod(1)];
moment_y = [moment_cross_up_fore(2), moment_cross_up_aft(2),moment_cross_low_fore(2), ...
    moment_cross_low_aft(2), moment_cross_pull_rod(2), moment_cross_tie_rod(2)];
moment_z = [moment_cross_up_fore(3), moment_cross_up_aft(3),moment_cross_low_fore(3), ...
    moment_cross_low_aft(3), moment_cross_pull_rod(3), moment_cross_tie_rod(3)];
applied_vector = [f_x(1), f_y(2), f_z(3), moment_applied(1), moment_applied(2), moment_applied(3)];
matrix_A = [sum_f_x; sum_f_y; sum_f_z; moment_x; moment_y; moment_z;];
forces_and_moments = applied_vector*inv(matrix_A)
%braking calculations
total_mass = 60*0.453592;
deltavelocity = [0:60]*0.44704;%change in speeds
stoptime = [0.1:0.01:1]; %time of braking force
numwheels = 4;  %number of wheels
for i = (1:length(deltavelocity))
    for j = (1:length(stoptime))
        f(i,j) = (total_mass*deltavelocity(i)/(stoptime(j)*numwheels));
    end 
end 
f_max_ary = max(f);
f_max = mean(f_max_ary,'all')/4;
f_up_fore = f_max.*up_fore
f_up_aft = f_max.*up_aft
f_low_fore = f_max.*low_fore
f_low_aft = f_max.*low_aft
f_pull_rod = f_max.*pull_rod
f_tie_rod = f_max.*tie_rod