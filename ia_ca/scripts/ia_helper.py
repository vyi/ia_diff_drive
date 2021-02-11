from math import pi, sin, cos, tan, atan2, sqrt
#from numpy import sqrt


def dist(p0, p1):
    x1 = p0[0] 
    x2 = p1[0]
    y1 = p0[1] 
    y2 = p1[1]
    return mysqrt((x1-x2)**2 + (y1-y2)**2)


def mysqrt(a):
    if a < 0:
        a = 0
    return sqrt(a)


def get_s_intersection(a,b):
    '''
    Calculates the intersection of sector a and b
    A sector is specified as a list [in_radius, out_radius, start_angle, end_angle]
    
    Returns
    -------
    The component sectors of a, which are unoccupied by sector b
    '''
    ans = []

    # Are the sectors really intersecting? 
    # If start_angle (or end_angle or both angles) of sector b fall inside the sector a start_angle - end_angle
    if not (angle_between(a[2],a[3],b[2]) or angle_between(a[2],a[3],b[3])):
        return [a]
    
    ## If we are here, there can be a valid intersection
    ## One condition where there won't be an intersection is when the outer radii of a is smaller
    ## than the inner radii of b. In this case b will be eniterly outside a.
    if (b[0] > a[1]):
        return [a]
    
    ## The sector before b
    if (b[2]-a[2] >0):
        temp_sector = [a[0],a[1],a[2],min(b[2],a[3])]
        ans.append(temp_sector)
    
    ## The sector after b
    if a[3]-b[3] >0:
        temp_sector = [a[0],a[1],max(b[3],a[2]),a[3]]
        ans.append(temp_sector)
    
    ## Sector inside
    temp_sector = [ a[0], min(b[0],a[1]), b[2], b[3]]
    ans.append(temp_sector)
    
    ## Sector outside
    if b[1] < a[1]:
        temp_sector = [b[1], a[1], b[2], b[3]]
        ans.append(temp_sector)
    
    return ans

## One way to check if a given angle falls within a given range 
def angle_between(start_angle, end_angle, check_angle):
    if end_angle < start_angle:
        end_angle = end_angle + pi*2
        
    if( check_angle >= start_angle) and (check_angle <= end_angle):
        return True
    
    return False

## Does the check_angle lie between lower_angle and upper_angle (?)
def check_contain(lower_angle,check_angle,upper_angle):
    l_angle = atan2(sin(lower_angle),cos(lower_angle))
    u_angle = atan2(sin(upper_angle),cos(upper_angle))
    c_angle = atan2(sin(check_angle),cos(check_angle))

    if (u_angle < l_angle):
        u_angle = u_angle + 2*pi
        if (c_angle < 0):
            c_angle = c_angle + 2*pi
 
    if (l_angle < c_angle and u_angle > c_angle):
        #status_contain = 1
        return 1
    else:
        #status_contain = 0;
        return 0



## Does an interval lie inside other interval ?
def inclusion_interval(I1_min_r,I1_max_r,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y):
    
    I1x1 = I1_x + I1_min_r * cos(I1_min_theta)
    I1y1 = I1_y + I1_min_r * sin(I1_min_theta)
    I1x1d = I1_x + I1_min_r * cos(I1_max_theta)
    I1y1d = I1_y + I1_min_r * sin(I1_max_theta)
    
    I1x2 = I1_x + I1_max_r * cos(I1_min_theta)
    I1y2 = I1_y + I1_max_r * sin(I1_min_theta)
    I1x2d = I1_x + I1_max_r * cos(I1_max_theta)
    I1y2d = I1_y + I1_max_r * sin(I1_max_theta)

    d_I2xy_I1x1y1 = abs(mysqrt((I2_x-I1x1)*(I2_x-I1x1) + (I2_y-I1y1)*(I2_y-I1y1)))
    a_I2xy_I1x1y1 = atan2((I1y1-I2_y),(I1x1-I2_x))
    d_I2xy_I1x1dy1d = abs(mysqrt((I2_x-I1x1d)*(I2_x-I1x1d) + (I2_y-I1y1d)*(I2_y-I1y1d)))
    a_I2xy_I1x1dy1d = atan2((I1y1d-I2_y),(I1x1d-I2_x))
    d_I2xy_I1x2y2 = abs(mysqrt((I2_x-I1x2)*(I2_x-I1x2) + (I2_y-I1y2)*(I2_y-I1y2)))
    a_I2xy_I1x2y2 = atan2((I1y2-I2_y),(I1x2-I2_x))
    d_I2xy_I1x2dy2d = abs(mysqrt((I2_x-I1x2d)*(I2_x-I1x2d) + (I2_y-I1y2d)*(I2_y-I1y2d)))
    a_I2xy_I1x2dy2d = atan2((I1y2d-I2_y),(I1x2d-I2_x))
    
    if(d_I2xy_I1x1y1 >= I2_min_r and d_I2xy_I1x1dy1d >= I2_min_r and d_I2xy_I1x2y2 >= I2_min_r and d_I2xy_I1x2dy2d >= I2_min_r and d_I2xy_I1x1y1 <= I2_max_r and d_I2xy_I1x1dy1d <= I2_max_r and d_I2xy_I1x2y2 <= I2_max_r and d_I2xy_I1x2dy2d <= I2_max_r): 
        if(check_contain(I2_min_theta,a_I2xy_I1x1y1,I2_max_theta) == 1 and check_contain(I2_min_theta,a_I2xy_I1x1dy1d,I2_max_theta) == 1 and check_contain(I2_min_theta,a_I2xy_I1x2y2,I2_max_theta) == 1 and check_contain(I2_min_theta,a_I2xy_I1x2dy2d,I2_max_theta) == 1):
            #status_inclusion = 1;
            return 1
        else:
            #status_inclusion = 0;
            return 0
    else:
        #status_inclusion = 0;
        return 0



## Given an Interval, split it into two intervals
def bisect(I_min,I_max):
    I_min_left = I_min
    I_max_left = I_min + (I_max-I_min)/2     # CORDIC is fine with divide-by-2 operation
    I_min_right = I_min + (I_max-I_min)/2   
    I_max_right = I_max
    return [I_min_left, I_max_left, I_min_right, I_max_right]


## Given two arcs, do they intersect ?
def arc_arc(x1,y1,theta1,theta1d,r1,x2,y2,theta2,theta2d,r2):
    arc1_circ2 = 0
    arc2_circ1 = 0
    arc1_arc2 = 0
    flag_theta1d_neg = 0
    flag_theta2d_neg = 0
    if theta1d<theta1:
        theta1d = theta1d + 2*pi
        flag_theta1d_neg = 1
    if theta2d<theta2:
        theta1d = theta1d + 2*pi
        flag_theta2d_neg = 1

    x1r = x1 +   r1 * cos(theta1)
    y1r = y1 +   r1 * sin(theta1)
    x1rd = x1 +  r1 * cos(theta1d)
    y1rd = y1 +  r1 * sin(theta1d)
    x1rdd = x1 + r1 * cos(theta1 + (theta1d-theta1)/2)
    y1rdd = y1 + r1 * sin(theta1 + (theta1d-theta1)/2)
    x2p = x2 +   r2 * cos(theta2)
    y2p = y2 +   r2 * sin(theta2)
    x2pd = x2 +  r2 * cos(theta2d)
    y2pd = y2 +  r2 * sin(theta2d)
    x2pdd = x2 + r2 * cos(theta2 + (theta2d-theta2)/2)
    y2pdd = y2 + r2 * sin(theta2 + (theta2d-theta2)/2)
    
    dist_x1y1_x2y2 =       mysqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2))
    dist_x1y1_x2py2p =     mysqrt((x1-x2p)*(x1-x2p) + (y1-y2p)*(y1-y2p))
    dist_x1y1_x2pdy2pd =   mysqrt((x1-x2pd)*(x1-x2pd) + (y1-y2pd)*(y1-y2pd))
    dist_x1y1_x2pddy2pdd = mysqrt((x1-x2pdd)*(x1-x2pdd) + (y1-y2pdd)*(y1-y2pdd))
    dist_x2y2_x1ry1r =     mysqrt((x1r-x2)*(x1r-x2) + (y1r-y2)*(y1r-y2))
    dist_x2y2_x1rdy1rd =   mysqrt((x1rd-x2)*(x1rd-x2) + (y1rd-y2)*(y1rd-y2))
    dist_x2y2_x1rddy1rdd = mysqrt((x1rdd-x2)*(x1rdd-x2) + (y1rdd-y2)*(y1rdd-y2))

    if dist_x1y1_x2y2 > (r1+r2):
            arc1_circ2 = 0
            arc2_circ1 = 0
            status_arc_arc = 0
    else:
            if (dist_x1y1_x2py2p < r1 and dist_x1y1_x2pdy2pd > r1) or (dist_x1y1_x2py2p > r1 and dist_x1y1_x2pdy2pd < r1):
                arc2_circ1 = 1 
            else:
                slope_x1y1_x2y2 = atan2((y1-y2),(x1-x2))
                if (flag_theta2d_neg == 1 and slope_x1y1_x2y2 < 0):
                    slope_x1y1_x2y2 = slope_x1y1_x2y2 + 2*pi
                if (dist_x1y1_x2py2p > r1 and dist_x1y1_x2pdy2pd > r1):
                    if (check_contain(theta2, slope_x1y1_x2y2, theta2d)):
                        arc2_circ1 = 2
                    else:
                        arc2_circ1 = 0
                elif (dist_x1y1_x2py2p < r1 and dist_x1y1_x2pdy2pd < r1):
                    if (check_contain(theta2+pi, slope_x1y1_x2y2, theta2d+pi)):
                        arc2_circ1 = 2
                    else:
                        arc2_circ1 = 0
                        
                

            if ((dist_x2y2_x1ry1r < r2 and dist_x2y2_x1rdy1rd > r2) or (dist_x2y2_x1ry1r > r2 and dist_x2y2_x1rdy1rd < r2)):
                arc1_circ2 = 1 
            else:
                slope_x2y2_x1y1 = atan2((y2-y1),(x2-x1))
                if (flag_theta1d_neg == 1 and slope_x2y2_x1y1 < 0):
                    slope_x2y2_x1y1 = slope_x2y2_x1y1 + 2*pi
                if (dist_x2y2_x1ry1r > r2 and dist_x2y2_x1rdy1rd > r2):
                    if (check_contain(theta1, slope_x2y2_x1y1, theta1d)):
                        arc1_circ2 = 2
                    else:
                        arc1_circ2 = 0
                elif ( dist_x2y2_x1ry1r < r2 and dist_x2y2_x1rdy1rd < r2):    
                    if ( check_contain( theta1 + pi, slope_x2y2_x1y1, theta1d + pi)):
                        arc1_circ2 = 2
                    else:
                        arc1_circ2 = 0
                        
                
                    
    
            if (arc2_circ1 == 2 and arc1_circ2 == 2):
                arc1_arc2 = 2
            elif ((arc2_circ1 == 2 and arc1_circ2 == 1) or (arc2_circ1 == 1 and arc1_circ2 == 2)):
                arc1_arc2 = 1
            elif (arc2_circ1 == 1 and arc1_circ2 == 1):
                angleI1 = atan2(sin((theta1+theta1d)/2), cos((theta1+theta1d)/2))
                angleI2 = atan2(sin((theta2+theta2d)/2), cos((theta2+theta2d)/2))
       
                angle_bw_centers = atan2((y2 - y1),(x2 - x1))
                angleI1 = atan2( sin(angleI1 - angle_bw_centers), cos(angleI1 - angle_bw_centers))
                angleI2 = atan2( sin(angleI2 - angle_bw_centers), cos(angleI2 - angle_bw_centers))
       
                if ((angleI1 > 0 and angleI1 < 180 and angleI2 > 0 and angleI2 < 180) or (angleI1 > -180 and angleI1 < 0 and angleI2 > -180 and angleI2 < 0)): 
                    arc1_arc2 = 1
                else:
                    arc1_arc2 = 0
                
       
    if (arc1_arc2 == 1 or arc1_arc2 == 2):
            #status_arc_arc = 1
        return 1
    else:
        #status_arc_arc = 0    
        return 0
## Given an arc and a line-segment, is there an intersection
def arc_line(x1,y1,theta1,theta1d,r1,x2,y2,theta2,r2l,r2u):
    if (theta1d<theta1):
        theta1d = theta1d + 2*pi
    
    x2p = x2 + r2u *  cos(theta2)
    y2p = y2 + r2u *  sin(theta2)
    x2pd = x2 + r2l * cos(theta2)
    y2pd = y2 + r2l * sin(theta2)

    x1r = x1 + r1 *  cos(theta1)
    y1r = y1 + r1 *  sin(theta1)
    x1rd = x1 + r1 * cos(theta1d)
    y1rd = y1 + r1 * sin(theta1d)
    
    line_arc =  0
    line_circ = 0

    l_perp = 0
    x1_lperp_q = 0
    y1_lperp_q = 0
    x1_lperp_qd = 0
    y1_lperp_qd = 0
    y1q=0
    x1q=0
    line_arc=0
    theta_x1y1_x1qy1q =0
    # (x2pd,y2pd) and (x2p,y2p) are the end points of the line
    # (x1,y1) is the center of the arc and (x1r,y1r),(x1rd,y1rd) are the
    # extreme points of the arc.

    d_x1y1_x2pdy2pd = mysqrt((x2pd-x1)*(x2pd-x1) + (y2pd-y1)*(y2pd-y1))
    d_x1y1_x2py2p =   mysqrt((x2p - x1)*(x2p - x1) + (y2p-y1)*(y2p-y1))

    if (d_x1y1_x2pdy2pd < r1 and d_x1y1_x2py2p > r1):
        line_circ = 1
        beta1 = atan2((y1r-y2pd),(x1r-x2pd))
        beta2 = atan2((y1rd-y2pd),(x1rd-x2pd))
        if (beta1>beta2):
            beta2 = beta2 + 2*pi
            if (theta2 < 0):
                alpha = theta2 + 2*pi
            else:
                alpha = theta2
        else:
            alpha = theta2
        
        if (alpha>beta1 and alpha<beta2):
            line_arc = 1
        else:
            line_arc = 0
        
    elif (d_x1y1_x2pdy2pd > r1 and d_x1y1_x2py2p < r1): # always you need to check beta1<theta2<beta2 condition the point of line segment which is inside the circle
        line_circ = 1
        alpha1 = theta2 + pi
        alpha1 = atan2(sin(alpha1), cos(alpha1))

        beta1 = atan2( (y1r-y2p) , (x1r-x2p) )
        beta2 = atan2( (y1rd-y2p), (x1rd-x2p) )
        if (beta1>beta2):
            beta2 = beta2+2*pi
            if (alpha1<0):
                alpha11 = alpha1 +2*pi
            else:
                alpha11 = alpha1
            
        else:
            alpha11 = alpha1
        
        if (alpha11>beta1 and alpha11<beta2):
            line_arc = 1
        else:
            line_arc = 0
        
    else:
        if ((d_x1y1_x2pdy2pd > r1 and d_x1y1_x2py2p > r1)):
            l_perp = abs((x1*tan(theta2) - y1 - tan(theta2)*x2 + y2)/(mysqrt( 1+tan(theta2)*tan(theta2))))
            x1_lperp_q = x1 + l_perp *  cos(theta2+pi/2)
            y1_lperp_q = y1 + l_perp *  sin(theta2+pi/2)
            x1_lperp_qd = x1 + l_perp * cos(theta2-pi/2)
            y1_lperp_qd = y1 + l_perp * sin(theta2-pi/2)

        if (-0.0001 < ((y1_lperp_q-y2)-tan(theta2)*(x1_lperp_q-x2)) and ((y1_lperp_q-y2)-tan(theta2)*(x1_lperp_q-x2)) < 0.0001):
            x1q = x1 + r1 * cos(theta2+pi/2)
            y1q = y1 + r1 * sin(theta2+pi/2)
            theta_x1y1_x1qy1q = atan2( sin(theta2+ pi/2), cos(theta2 + pi/2))
        elif(-0.0001 < ((y1_lperp_qd-y2)- tan(theta2)*(x1_lperp_qd-x2)) and ((y1_lperp_qd-y2) - tan(theta2)*(x1_lperp_qd-x2)) < 0.0001):
            x1q = x1 + r1 * cos(theta2 - pi/2)
            y1q = y1 + r1 * sin(theta2 - pi/2)
            theta_x1y1_x1qy1q = atan2( sin(theta2-pi/2), cos(theta2- pi/2))
        
        d_x2y2_lperp =   mysqrt(d_x1y1_x2pdy2pd * d_x1y1_x2pdy2pd - l_perp * l_perp)
        d_x2py2p_lperp = mysqrt(d_x1y1_x2py2p * d_x1y1_x2py2p - l_perp * l_perp)
        
        if(d_x2y2_lperp < (r2u-r2l) and d_x2py2p_lperp < (r2u-r2l) and l_perp < r1):
            line_circ = 2
            if (((x1r*tan(theta2)-y1r-tan(theta2)*x2+y2)<0 and (x1rd*tan(theta2)-y1rd-tan(theta2)*x2+y2)>0) or ((x1r*tan(theta2)-y1r-tan(theta2)*x2+y2)>0 and (x1rd*tan(theta2)-y1rd-tan(theta2)*x2+y2)<0)):
                line_arc = 1
            else: 
                if (((x1r*tan(theta2)-y1r-tan(theta2)*x2+y2)<0 and (x1q*tan(theta2)-y1q-tan(theta2)*x2+y2)>0) or ((x1r*tan(theta2)-y1r-tan(theta2)*x2+y2)>0 and (x1q*tan(theta2)-y1q-tan(theta2)*x2+y2)<0)):
    #                  if(theta1<180 && theta1d>180)
    #                      if(theta_x1y1_x1qy1q < 0)
    #                          theta_x1y1_x1qy1q = theta_x1y1_x1qy1q +360;
    #                      end
    #                  end
    #                if(theta_x1y1_x1qy1q > theta1 && theta_x1y1_x1qy1q < theta1d)
                     if (check_contain(theta1,theta_x1y1_x1qy1q,theta1d) == 1):   
                         line_arc = 2
                     else:
                         line_arc = 0
                        

                else: 
                    line_arc = 0
            
            
        else:
            line_circ = 0
            line_arc = 0
        
        
    

    if(line_arc == 1 or line_arc == 2):
        #status_arc_line = 1;
        return 1
    else:
        #status_arc_line = 0;
        return 0



## Given two line segments, ref cone vertex point (x1,y1), angle with horizon (theta1) and
##    the radii (r1l, r1u) 
def line_line(x1,y1,theta1,r1l,r1u,x2,y2,theta2,r2l,r2u):
    ## Get the end points for segment 1
    x1ru = x1 + r1u*cos(theta1)
    y1ru = y1 + r1u*sin(theta1)
    x1rl = x1 + r1l*cos(theta1)
    y1rl = y1 + r1l*sin(theta1)

    ## Get the end points for segment 2
    x2ru = x2 + r2u*cos(theta2)
    y2ru = y2 + r2u*sin(theta2)
    x2rl = x2 + r2l*cos(theta2)
    y2rl = y2 + r2l*sin(theta2)

    ## Get slopes these segments
    slope_x1y1_x1ry1r = atan2((y1ru-y1),(x1ru-x1))
    slope_x2y2_x2ry2r = atan2((y2ru-y2),(x2ru-x2))

    if((slope_x1y1_x1ry1r == slope_x2y2_x2ry2r) or (slope_x1y1_x1ry1r + pi == slope_x2y2_x2ry2r) or (slope_x1y1_x1ry1r == slope_x2y2_x2ry2r + pi)):
        #status_line_line = 0
        return 0
    else:
        if((((y1ru-y1)*(x2rl-x1) - (x1ru-x1)*(y2rl-y1))>0 and ((y1ru-y1)*(x2ru-x1) - (x1ru-x1)*(y2ru-y1))<0) or (((y1ru-y1)*(x2rl-x1) - (x1ru-x1)*(y2rl-y1))<0 and ((y1ru-y1)*(x2ru-x1) - (x1ru-x1)*(y2ru-y1))>0)):
            if((((y2ru-y2)*(x1rl-x2) - (x2ru-x2)*(y1rl-y2))>0 and ((y2ru-y2)*(x1ru-x2) - (x2ru-x2)*(y1ru-y2))<0) or (((y2ru-y2)*(x1rl-x2) - (x2ru-x2)*(y1rl-y2))<0 and ((y2ru-y2)*(x1ru-x2) - (x2ru-x2)*(y1ru-y2))>0)):
                #status_line_line = 1
                return 1
            else:
                #status_line_line = 0
                return 0
        else:
            #status_line_line = 0
            return 0





## Given two robot intervals, is there a collision ?
def collision_detection(I1_min_r,I1_max_r,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y):


    status_inter_line1 = line_line(I1_x,I1_y,I1_min_theta,I1_min_r,I1_max_r,I2_x,I2_y,I2_min_theta,I2_min_r,I2_max_r)
    status_inter_line2 = line_line(I1_x,I1_y,I1_min_theta,I1_min_r,I1_max_r,I2_x,I2_y,I2_max_theta,I2_min_r,I2_max_r)
    status_inter_line3 = line_line(I1_x,I1_y,I1_max_theta,I1_min_r,I1_max_r,I2_x,I2_y,I2_min_theta,I2_min_r,I2_max_r)
    status_inter_line4 = line_line(I1_x,I1_y,I1_max_theta,I1_min_r,I1_max_r,I2_x,I2_y,I2_max_theta,I2_min_r,I2_max_r)
    
    status_inter_arcu_arcu = arc_arc(I1_x,I1_y,I1_min_theta,I1_max_theta,I1_max_r,I2_x,I2_y,I2_min_theta,I2_max_theta,I2_max_r)
    status_inter_arcu_arcl = arc_arc(I1_x,I1_y,I1_min_theta,I1_max_theta,I1_max_r,I2_x,I2_y,I2_min_theta,I2_max_theta,I2_min_r)
    status_inter_arcl_arcu = arc_arc(I1_x,I1_y,I1_min_theta,I1_max_theta,I1_min_r,I2_x,I2_y,I2_min_theta,I2_max_theta,I2_max_r)
    status_inter_arcl_arcl = arc_arc(I1_x,I1_y,I1_min_theta,I1_max_theta,I1_min_r,I2_x,I2_y,I2_min_theta,I2_max_theta,I2_min_r)

    status_inter_arc_line1 = arc_line(I1_x,I1_y,I1_min_theta,I1_max_theta,I1_max_r,I2_x,I2_y,I2_min_theta,I2_min_r,I2_max_r)
    status_inter_arc_line2 = arc_line(I1_x,I1_y,I1_min_theta,I1_max_theta,I1_max_r,I2_x,I2_y,I2_max_theta,I2_min_r,I2_max_r)
    status_inter_arc_line3 = arc_line(I2_x,I2_y,I2_min_theta,I2_max_theta,I2_max_r,I1_x,I1_y,I1_min_theta,I1_min_r,I1_max_r)
    status_inter_arc_line4 = arc_line(I2_x,I2_y,I2_min_theta,I2_max_theta,I2_max_r,I1_x,I1_y,I1_max_theta,I1_min_r,I1_max_r)

    status_inter_arc_line5 = arc_line(I1_x,I1_y,I1_min_theta,I1_max_theta,I1_min_r,I2_x,I2_y,I2_min_theta,I2_min_r,I2_max_r)
    status_inter_arc_line6 = arc_line(I1_x,I1_y,I1_min_theta,I1_max_theta,I1_min_r,I2_x,I2_y,I2_max_theta,I2_min_r,I2_max_r)
    status_inter_arc_line7 = arc_line(I2_x,I2_y,I2_min_theta,I2_max_theta,I2_min_r,I1_x,I1_y,I1_min_theta,I1_min_r,I1_max_r)
    status_inter_arc_line8 = arc_line(I2_x,I2_y,I2_min_theta,I2_max_theta,I2_min_r,I1_x,I1_y,I1_max_theta,I1_min_r,I1_max_r)
        
    if((status_inter_line1 == 1) or (status_inter_line2 == 1) or (status_inter_line3 == 1) or (status_inter_line4 == 1) or (status_inter_arcu_arcu == 1) or (status_inter_arcu_arcl == 1) or (status_inter_arcl_arcu == 1) or (status_inter_arcl_arcl == 1) or (status_inter_arc_line1 == 1) or (status_inter_arc_line2 == 1) or (status_inter_arc_line3 == 1) or (status_inter_arc_line4 == 1) or (status_inter_arc_line5 == 1) or (status_inter_arc_line6 == 1) or (status_inter_arc_line7 == 1) or (status_inter_arc_line8 == 1)):
        #collision_status = 1
        return 1
    else:
        bot_inside_obs = inclusion_interval(I1_min_r,I1_max_r,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y)
        obs_inside_bot = inclusion_interval(I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y,I1_min_r,I1_max_r,I1_min_theta,I1_max_theta,I1_x,I1_y)

        if(bot_inside_obs == 1 or obs_inside_bot == 1):
            #collision_status = 1
            return 1
        else:
            #collision_status = 0
            return 0




## When there is a collision... how do we avoid collision ?
def collision_avoidance(I1_min_r,I1_max_r,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y):
    I_obs_min_r = 0
    I_obs_max_r = 0
    I_obs_min_theta = 0
    I_obs_max_theta = 0
    k=0
    h=0
    N=10
    M=10
    I_robot_free = []
    
    if(I1_min_theta > I1_max_theta):
        I1_max_theta = I1_max_theta + 2*pi
    

    if(I2_min_theta > I2_max_theta):
        I2_max_theta = I2_max_theta + 2*pi
    
    [I_min_left, I_max_left, I_min_right, I_max_right] = bisect(I1_min_r,I1_max_r)

    collision_status_left = collision_detection(I_min_left,I_max_left,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y)
    collision_status_right = collision_detection(I_min_right,I_max_right,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y)

    while (k < N): 
        if ((collision_detection(I_min_right,I_max_right,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 0) and (collision_detection(I_min_left,I_max_left,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1)):
            [ I_min_leftleft, I_max_leftleft, I_min_leftright, I_max_leftright] = bisect(I_min_left,I_max_left)
            I_min_left1 = I_min_leftleft
            I_max_left1 = I_max_leftleft
            I_min_right1 = I_min_leftright
            I_max_right1 = I_max_leftright
        elif ((collision_detection(I_min_left,I_max_left,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 0) and (collision_detection(I_min_right,I_max_right,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1)):
            [I_min_rightleft,I_max_rightleft,I_min_rightright,I_max_rightright] = bisect(I_min_right,I_max_right)    
            I_min_left1 = I_min_rightleft
            I_max_left1 = I_max_rightleft
            I_min_right1 = I_min_rightright
            I_max_right1 = I_max_rightright
        else:

            [I_min_leftleft, I_max_leftleft, I_min_leftright, I_max_leftright] = bisect(I_min_left,I_max_left)
            [I_min_rightleft, I_max_rightleft, I_min_rightright, I_max_rightright] = bisect(I_min_right,I_max_right)

            if(collision_detection(I_min_leftleft,I_max_leftleft,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1):
                I_min_left1 = I_min_leftleft
                I_max_left1 = I_max_leftleft
        #          plot_sec(I1_min_theta,I1_max_theta,I1_x,I1_y,I_min_left1,I_max_left1,'m');
            else:
                I_min_left1 = I_min_leftright
                I_max_left1 = I_max_leftright
        #          plot_sec(I1_min_theta,I1_max_theta,I1_x,I1_y,I_min_left1,I_max_left1,'y');
            
            if(collision_detection(I_min_rightright,I_max_rightright,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1):
                I_min_right1 = I_min_rightright
                I_max_right1 = I_max_rightright
        #          plot_sec(I1_min_theta,I1_max_theta,I1_x,I1_y,I_min_right1,I_max_right1,'b');
            else:
                I_min_right1 = I_min_rightleft
                I_max_right1 = I_max_rightleft
        #          plot_sec(I1_min_theta,I1_max_theta,I1_x,I1_y,I_min_right1,I_max_right1,'c');
            
        I_min_left = I_min_left1
        I_max_left = I_max_left1
        I_min_right = I_min_right1
        I_max_right = I_max_right1
        k = k+1
    
    I_obs_min_r = I_min_left
    I_obs_max_r = I_max_right

    # bisection w.r.t theta
    
    [I_min_left, I_max_left, I_min_right, I_max_right] = bisect(I1_min_theta,I1_max_theta)
    collision_status_left = collision_detection(I1_min_r,I1_max_r,I_min_left,I_max_left,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y)
    collision_status_right = collision_detection(I1_min_r,I1_max_r,I_min_right,I_max_right,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y)

    
    while(h < M): # bisction w.r.t theta
        if((collision_detection(I1_min_r,I1_max_r,I_min_right,I_max_right,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 0) and (collision_detection(I1_min_r,I1_max_r,I_min_left,I_max_left,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1)):
            [I_min_leftleft, I_max_leftleft, I_min_leftright, I_max_leftright] = bisect(I_min_left,I_max_left)
            I_min_left1 = I_min_leftleft
            I_max_left1 = I_max_leftleft
            I_min_right1 = I_min_leftright
            I_max_right1 = I_max_leftright
        elif((collision_detection(I1_min_r,I1_max_r,I_min_left,I_max_left,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 0) and (collision_detection(I1_min_r,I1_max_r,I_min_right,I_max_right,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1)):
            [I_min_rightleft,I_max_rightleft,I_min_rightright,I_max_rightright] = bisect(I_min_right,I_max_right)    
            I_min_left1 = I_min_rightleft
            I_max_left1 = I_max_rightleft
            I_min_right1 = I_min_rightright
            I_max_right1 = I_max_rightright
        else:
            [I_min_leftleft,  I_max_leftleft, I_min_leftright, I_max_leftright] = bisect(I_min_left,I_max_left)
            [I_min_rightleft, I_max_rightleft, I_min_rightright, I_max_rightright] = bisect(I_min_right,I_max_right)

            if(collision_detection(I1_min_r,I1_max_r,I_min_leftleft,I_max_leftleft,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1):
                I_min_left1 = I_min_leftleft
                I_max_left1 = I_max_leftleft
    #             plot_sec(I_min_left1,I_max_left1,I1_x,I1_y,I1_min_r,I1_max_r,'m');    
            else:
                I_min_left1 = I_min_leftright
                I_max_left1 = I_max_leftright
    #         plot_sec(I_min_left1,I_max_left1,I1_x,I1_y,I1_min_r,I1_max_r,'y');
        
            if(collision_detection(I1_min_r,I1_max_r,I_min_rightright,I_max_rightright,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y) == 1):
                I_min_right1 = I_min_rightright
                I_max_right1 = I_max_rightright
    #         plot_sec(I_min_right1,I_max_right1,I1_x,I1_y,I1_min_r,I1_max_r,'b');
            else:
                I_min_right1 = I_min_rightleft
                I_max_right1 = I_max_rightleft
    #         plot_sec(I_min_right1,I_max_right1,I1_x,I1_y,I1_min_r,I1_max_r,'c');
        
        I_min_left = I_min_left1
        I_max_left = I_max_left1
        I_min_right = I_min_right1
        I_max_right = I_max_right1
        h = h+1
    
    I_obs_min_theta = I_min_left
    I_obs_max_theta = I_max_right
    
    I_robot_obs = [I_obs_min_r, I_obs_max_r, I_obs_min_theta, I_obs_max_theta]
    #I_robot_free = free_sectors(I1_min_r,I1_max_r,I1_min_theta,I1_max_theta,I1_x,I1_y,I2_min_r,I2_max_r,I2_min_theta,I2_max_theta,I2_x,I2_y,I_robot_obs)
    #return I_robot_free
    return I_robot_obs



if __name__ == '__main__':
    print("This is a helper module")
