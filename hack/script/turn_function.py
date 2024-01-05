import numpy
import geopandas as gpd
from shapely.geometry import Point, LineString, Polygon
import math
import fiona
from shapely.geometry import Point
import utm
import numpy as np
import math as mt
from shapely.ops import unary_union

fiona.supported_drivers

from geopandas import GeoSeries

def define_turns_circle(ori, des, ref, r=50, turn_radius=4, ajust_rank=0):
    """ Draw turn from ori to des. ref is supposed to be the other side of des row. 
    Serves as a reference. Number of point, deepnest of turn and number of points before turn 
    to be chosen. Need the dataframe to have : 'end_side', 'Comment', 'id', 'id_rg', 'Zmsl'.

    Parameters
    --------
    ori : gdf of 1 point
        Point of origine of the turn
    des: gdf of 1 point
        Destination of the turn
    ref: gdf of 1 point
        Reference. Used to get the third dimension
    r: int, default 50
        Nb of minimum points in the turn. There is a bonus serie of points to smooth the 
        begining and end of turn. 
    turn_radius: int, default 4
        Number of meter deep the turn is. Expressed in portion of a meter from segment des-ref.
    ajust_rank: int, default 0
        Number of half meter bonus betwen the end of row (des) and the beggining the the turn. 
        It is a bonus, 4 already included. 
        Expressed in portions of half a meter from segment des-ref

    Returns
    --------
    geodataframe with the points of the turn in the index and id order from ori to des. 
    Mind the fact that ori, res and ref are not included in the return

    """
    pr = 1/r    # Still no a plan where to hide it!

    # Check the end_side
    if ori['end_side'].values[0] != des['end_side'].values[0]:
        print("Wait, the end_side aren't identical!")
        return "Wait, the end_side aren't identical!"
    else:
        es = ori['end_side'].values[0]
    # If turn radius to hight, we will have a sin like problem sooo....
    if turn_radius > 4:
        pow_index = 4
    else:
        pow_index = 4
    # Calculating row length in a way or another:
    delx = des.geometry.x.values[0]-ref.geometry.x.values[0]
    dely = des.geometry.y.values[0]-ref.geometry.y.values[0]    

    # Check CRS and change length according to it.
    if ori.geometry.crs == "RGF93 v1 / Lambert-93":
        lon = LineString([des.geometry.values[0],
                         ref.geometry.values[0]]).length
        crs = "RGF93 v1 / Lambert-93"
    else:
        print("Carefull mate, the SCR is not expected to be like this! \
            Issue in define_turns fonction!")

    # Creation of in-between calcul
    dx1 = des.geometry.x.values[0]
    dx2 = ori.geometry.x.values[0]
    dy1 = des.geometry.y.values[0]
    dy2 = ori.geometry.y.values[0]
    # if a point has been switched by a fonction, the unchanged point move 2m out the row to smooth the turn. 
    if ori.Comment.values[0] != des.Comment.values[0]:
        if ori.Comment.values[0] == "switched":
            dx1 = (des.geometry.x.values[0] + 2*delx/lon)
            dy1 = (des.geometry.y.values[0] + 2*dely/lon)
        elif des.Comment.values[0] == "switched":
            dx2 = (ori.geometry.x.values[0] + 2*delx/lon)
            dy2 = (ori.geometry.y.values[0] + 2*dely/lon)
    dx = dx1 - dx2
    dy = dy1 - dy2

    # Initialization of dict
    position = { "id_rg": [], "end_side": [], "geometry": [], "Zmsl": []}

    # Safety extend origin if requested,  if there is a safe area, not needed :
    if ori.Comment.values[0] == "switched":
        coord_x = ori.geometry.x.values[0]
        coord_y = ori.geometry.y.values[0]
    else:
        for f in range(1, 4+ajust_rank):
            # The coord of departur point
            coord_x = ori.geometry.x.values[0]+f*.5*delx/(lon)
            coord_y = ori.geometry.y.values[0]+f*.5*dely/(lon)
            # Append the safety points point
#            position["id"].append(ori["id"].values[0])
            position["id_rg"].append(ori["id_rg"].values[0])
            position["end_side"].append(ori["end_side"].values[0])
            position["geometry"].append(Point(coord_x, coord_y))
            position["Zmsl"].append(ori["Zmsl"].values[0])

    # And them...
    for l in range(r):
        # Extrapolation point position
        # Extrapolation X and Y
        if l < r/10 or l > 9*r/10:
            for i in range(3):
                if l < r/10:
                    x = (l*10+2+i*3)*pr/10
                elif l > 9*r/10:
                    x = (l*10+3+i*3)*pr/10
                cor_X = dx*x + turn_radius*math.pow(math.sin(x*math.pi),
                                                    1/pow_index)*delx/lon
                cor_Y = dy*x + turn_radius*math.pow(math.sin(x*math.pi),
                                                    1/pow_index)*dely/lon
                end_side_tp = es
                new_coord_x = coord_x + cor_X
                new_coord_y = coord_y + cor_Y
#                position["id"].append(ori["id"].values[0] + l)
                position["id_rg"].append(None)
                position["end_side"].append(end_side_tp)
                position["geometry"].append(Point(new_coord_x, new_coord_y))
                position["Zmsl"].append(numpy.mean([ori["Zmsl"].values[0],
                                                    des["Zmsl"].values[0]]))
        elif l <= r:
            x = (l+1)*pr
            cor_X = dx*x + turn_radius*math.pow(math.sin(x*math.pi),
                                                1/pow_index)*delx/lon
            cor_Y = dy*x + turn_radius*math.pow(math.sin(x*math.pi),
                                                1/pow_index)*dely/lon
            end_side_tp = es
            new_coord_x = coord_x + cor_X
            new_coord_y = coord_y + cor_Y
#            position["id"].append(ori["id"].values[0] + l)
            position["id_rg"].append(None)
            position["end_side"].append(end_side_tp)
            position["geometry"].append(Point(new_coord_x, new_coord_y))
            position["Zmsl"].append(numpy.mean([ori["Zmsl"].values[0],
                                                des["Zmsl"].values[0]]))

    # Safety extend end if requested:
    if des.Comment.values[0] == "switched":
        coord_x = des.geometry.x.values[0]
        coord_y = des.geometry.y.values[0]
    else:
        for f in range(3 + ajust_rank, 0, -1):
            # The coord of departur point
            coord_x = des.geometry.x.values[0]+f*.5*delx/(lon)
            coord_y = des.geometry.y.values[0]+f*.5*dely/(lon)
            # Append the safety points point
#            position["id"].append(des["id"].values[0])
            position["id_rg"].append(des["id_rg"].values[0])
            position["end_side"].append(des["end_side"].values[0])
            position["geometry"].append(Point(coord_x, coord_y))
            position["Zmsl"].append(des["Zmsl"].values[0])

    puntos_infernos = gpd.geodataframe.GeoDataFrame(position,
                                                    geometry="geometry",
                                                    crs=crs)
    return puntos_infernos

# #############################################################################
# ###############Fonction define_turns_bis that turns points into irregulars turn
# #############################################################################

def define_turns_irr(ori, des, ref, r=30, turn_radius=4, ajust_rank=0, type_turn = 'normal', pow_index = 2):
    """ Draw turn from ori to des. ref is supposed to be the other side of des row. Serves as a reference.
    Number of point, deepnest of turn and number of points before turn to be chosen. Need the dataframe to
    have : 'end_side', 'Comment', 'id', 'id_rg', 'Zmsl'.

    Parameters
    --------
    ori : gdf of 1 point
        Point of origine of the turn
    des: gdf of 1 point
        Destination of the turn
    ref: gdf of 1 point
        Reference. Used to get the third dimension
    r: int, default 50
        Nb of minimum points in the turn. There is a bonus serie of points to smooth the begining and end of turn.
    turn_radius: int, default 4
        Number of meter deep the turn is. Expressed in portion of a meter from segment des-ref.
    ajust_rank: int, default 0
        Number of half meter bonus betwen the end of row (des) and the beggining the the turn. It is a bonus, 4 already included.
        Expressed in portions of half a meter from segment des-ref

    Returns
    --------
    geodataframe with the points of the turn in the index and id order from ori to des. Mind the fact that ori, res and ref are not included in the return

    """
    pr = 1/r    # still no a plan where to hide it!

    # Check the end_side
    if ori['end_side'].values[0] != des['end_side'].values[0]:
        print("Wait, the end_side aren't identical!")
        return "Wait, the end_side aren't identical!"
    else:
        es = ori['end_side'].values[0]
    # If turn radius to hight, we will have a sin like problem sooo....
    if turn_radius > 4 and pow_index == 2:
        pow_index = 4
    elif pow_index != 2:
        pow_index = pow_index
    else:
        pow_index = 2
    # Calculating row length in a way or another:
    delx = des.geometry.x.values[0]-ref.geometry.x.values[0]
    dely = des.geometry.y.values[0]-ref.geometry.y.values[0]
   # pow_index = 6    ##test by franklin
    #ori.set_geometry("geometry")     #test 

    # Check CRS and change length according to it.
    if ori.geometry.crs == "EPSG:2154":
        lon = LineString([des.geometry.values[0],
                         ref.geometry.values[0]]).length
        crs = "EPSG:2154"
    else:
        #lon = LineString([des.geometry.values[0],
         #                 ref.geometry.values[0]]).length
        #crs = "RGF93"   ##modifier pour test
        print("Carefull mate, the SCR is not expected to be like this! \
            Issue in define_turns fonction!")


    # Creation of in-between calcul
    dx1 = des.geometry.x.values[0]
    dx2 = ori.geometry.x.values[0]
    dy1 = des.geometry.y.values[0]
    dy2 = ori.geometry.y.values[0]
    # If the point has been switched by a fonction, the point move a little bit
    if ori.Comment.values[0] != des.Comment.values[0]:
        if ori.Comment.values[0] == "switched":
            dx1 = (des.geometry.x.values[0] + 2*delx/lon)
            dy1 = (des.geometry.y.values[0] + 2*dely/lon)         #### Commenter pour tester turn_traj
        elif des.Comment.values[0] == "switched":
            dx2 = (ori.geometry.x.values[0] + 2*delx/lon)
            dy2 = (ori.geometry.y.values[0] + 2*dely/lon)
    dx = dx1 - dx2
    dy = dy1 - dy2

    # Initialization of dict
    position = {"id_rg": [], "end_side": [], "geometry": [], "Zmsl": []}

    # Safety extend origin if requested,  if there is a safe area, not needed :
    if ori.Comment.values[0] == "switched":
        coord_x = ori.geometry.x.values[0]
        coord_y = ori.geometry.y.values[0]
    else:
        for f in range(1, 4 + ajust_rank):
            # The coord of departur point
            coord_x = ori.geometry.x.values[0] + f*.5*delx/(lon) #+ f*.01*-dx
            coord_y = ori.geometry.y.values[0] + f*.5*dely/(lon) #+ f*.01*-dy
            # Append the safety points point
  #          position["id"].append(ori["id"].values[0])
            position["id_rg"].append(ori["id_rg"].values[0])
            position["end_side"].append(ori["end_side"].values[0])
            position["geometry"].append(Point(coord_x, coord_y))
            position["Zmsl"].append(ori["Zmsl"].values[0])

    # And them...
    for l in range(r):
        # Extrapolation point position
        # Extrapolation X and Y
        if l < r/10 or l > 9*r/10:
            for i in range(3):
                if l < r/10:
                    x = (l*10+2+i*3)*pr/10
                elif l > 9*r/10:
                    x = (l*10+3+i*3)*pr/10
                # Change type of turn according to requested
                if type_turn == 'normal':
                    cor_X = dx*x + turn_radius*math.pow(math.sin(x*(math.pi)), 1/pow_index)*delx/lon
                    cor_Y = dy*x + turn_radius*math.pow(math.sin(x*(math.pi)), 1/pow_index)*dely/lon
                elif type_turn == 'early':
                    cor_X = dx*x + turn_radius*(((math.cos(x*math.pi)+1)*2/5)+ math.pow(math.sin(x*math.pi),1/pow_index)*3/2)/2*delx/lon
                    cor_Y = dy*x + turn_radius*(((math.cos(x*math.pi)+1)*2/5)+ math.pow(math.sin(x*math.pi),1/pow_index)*3/2)/2*dely/lon
                elif type_turn == 'late':
                    cor_X = dx*x + turn_radius*(((math.cos(x*math.pi+math.pi)+1)*2/5)+ math.pow(math.sin(x*math.pi),1/pow_index)*3/2)/2*delx/lon
                    cor_Y = dy*x + turn_radius*(((math.cos(x*math.pi+math.pi)+1)*2/5)+ math.pow(math.sin(x*math.pi),1/pow_index)*3/2)/2*dely/lon
                end_side_tp = es
                new_coord_x = coord_x + cor_X
                new_coord_y = coord_y + cor_Y
#                position["id"].append(ori["id"].values[0] + l)
                position["id_rg"].append(None)
                position["end_side"].append(end_side_tp)
                position["geometry"].append(Point(new_coord_x, new_coord_y))
                position["Zmsl"].append(numpy.mean([ori["Zmsl"].values[0],
                                                    des["Zmsl"].values[0]]))
        elif l <= r:
            x = (l+1)*pr
            # Change type of turn according to requested
            if type_turn == 'normal':
                cor_X = dx*x + turn_radius*math.pow(math.sin(x*(math.pi)), 1/pow_index)*delx/lon
                cor_Y = dy*x + turn_radius*math.pow(math.sin(x*(math.pi)), 1/pow_index)*dely/lon
            elif type_turn == 'early':
                cor_X = dx*x + turn_radius*(((math.cos(x*math.pi)+1)*2/5)+ math.pow(math.sin(x*math.pi),1/pow_index)*3/2)/2*delx/lon
                cor_Y = dy*x + turn_radius*(((math.cos(x*math.pi)+1)*2/5)+ math.pow(math.sin(x*math.pi),1/pow_index)*3/2)/2*dely/lon
            elif type_turn == 'late':
                cor_X = dx*x + turn_radius*(((math.cos(x*math.pi+math.pi)+1)*2/5)+ math.pow(math.sin(x*math.pi),1/pow_index)*3/2)/2*delx/lon
                cor_Y = dy*x + turn_radius*(((math.cos(x*math.pi+math.pi)+1)*2/5)+ math.pow(math.sin(x*math.pi),1/pow_index)*3/2)/2*dely/lon
            end_side_tp = es
            new_coord_x = coord_x + cor_X
            new_coord_y = coord_y + cor_Y
#            position["id"].append(ori["id"].values[0] + l)
            position["id_rg"].append(None)
            position["end_side"].append(end_side_tp)
            position["geometry"].append(Point(new_coord_x, new_coord_y))
            position["Zmsl"].append(numpy.mean([ori["Zmsl"].values[0],
                                                des["Zmsl"].values[0]]))

    # Safety extend end if requested:
    if des.Comment.values[0] == "switched":
        coord_x = des.geometry.x.values[0]
        coord_y = des.geometry.y.values[0]
    else:
        for f in range(3 + ajust_rank, 0, -1):
            # The coord of departur point
            coord_x = des.geometry.x.values[0] + f*.5*delx/(lon) #+ f*.01*dx
            coord_y = des.geometry.y.values[0] + f*.5*dely/(lon) #+ f*.01*dy
            # Append the safety points point
#            position["id"].append(des["id"].values[0])
            position["id_rg"].append(des["id_rg"].values[0])
            position["end_side"].append(des["end_side"].values[0])
            position["geometry"].append(Point(coord_x, coord_y))
            position["Zmsl"].append(des["Zmsl"].values[0])

    puntos_infernos = gpd.geodataframe.GeoDataFrame(position,
                                                    geometry="geometry",
                                                    crs=crs)
    return puntos_infernos

# ##############################################################################
# ################################### Define turns in Curly Bracket
# ##############################################################################

def define_turns_curly(ori, des, ref, r=50, turn_radius=4, ajust_rank=0):
    """ Draw turn from ori to des. ref is supposed to be the other side of des row. 
    Serves as a reference. Number of point, deepnest of turn and number of points before turn 
    to be chosen. Need the dataframe to have : 'end_side', 'Comment', 'id', 'id_rg', 'Zmsl'.

    Parameters
    --------
    ori : gdf of 1 point
        Point of origine of the turn
    des: gdf of 1 point
        Destination of the turn
    ref: gdf of 1 point
        Reference. Used to get the third dimension
    r: int, default 50
        Nb of minimum points in the turn. There is a bonus serie of points to smooth the 
        begining and end of turn. 
    turn_radius: int, default 4
        Number of meter deep the turn is. Expressed in portion of a meter from segment des-ref.
    ajust_rank: int, default 0
        Number of half meter bonus betwen the end of row (des) and the beggining the the turn. 
        It is a bonus, 4 already included. 
        Expressed in portions of half a meter from segment des-ref

    Returns
    --------
    geodataframe with the points of the turn in the index and id order from ori to des. 
    Mind the fact that ori, res and ref are not included in the return

    """
    pr = 1/r    # Still no a plan where to hide it!

    # Check the end_side
    if ori['end_side'].values[0] != des['end_side'].values[0]:
        print("Wait, the end_side aren't identical!")
        return "Wait, the end_side aren't identical!"
    else:
        es = ori['end_side'].values[0]
    # If turn radius to hight, we will have a sin like problem sooo....
    if turn_radius > 4:
        pow_index = 4
    else:
        pow_index = 2
    # Calculating row length in a way or another:
    delx = des.geometry.x.values[0]-ref.geometry.x.values[0]
    dely = des.geometry.y.values[0]-ref.geometry.y.values[0]    

    # Check CRS and change length according to it.
    if ori.geometry.crs == "EPSG:2154":
        lon = LineString([des.geometry.values[0],
                         ref.geometry.values[0]]).length
        crs = "EPSG:2154"
    else:
        print("Carefull mate, the SCR is not expected to be like this! \
            Issue in define_turns fonction!")

    # Creation of in-between calcul
    dx1 = des.geometry.x.values[0]
    dx2 = ori.geometry.x.values[0]
    dy1 = des.geometry.y.values[0]
    dy2 = ori.geometry.y.values[0]
    # if the point has been switched by a fonction, the point move a little bit
    if ori.Comment.values[0] != des.Comment.values[0]:
        if ori.Comment.values[0] == "switched":
            dx1 = (des.geometry.x.values[0] + 2*delx/lon)
            dy1 = (des.geometry.y.values[0] + 2*dely/lon)
        elif des.Comment.values[0] == "switched":
            dx2 = (ori.geometry.x.values[0] + 2*delx/lon)
            dy2 = (ori.geometry.y.values[0] + 2*dely/lon)
    dx = dx1 - dx2
    dy = dy1 - dy2

    # Initialization of dict
    position = {"id": [], "id_rg": [], "end_side": [], "geometry": [], "Zmsl": []}

    # Safety extend origin if requested,  if there is a safe area, not needed :
    if ori.Comment.values[0] == "switched":
        coord_x = ori.geometry.x.values[0]
        coord_y = ori.geometry.y.values[0]
    else:
        for f in range(1, 4+ajust_rank):
            # The coord of departur point
            coord_x = ori.geometry.x.values[0]+f*.5*delx/(lon)
            coord_y = ori.geometry.y.values[0]+f*.5*dely/(lon)
            # Append the safety points point
#            position["id"].append(ori["id"].values[0])
            position["id_rg"].append(ori["id_rg"].values[0])
            position["end_side"].append(ori["end_side"].values[0])
            position["geometry"].append(Point(coord_x, coord_y))
            position["Zmsl"].append(ori["Zmsl"].values[0])

    # And them...
    for l in range(r):
        # Extrapolation point position
        # Extrapolation X and Y
        """if l < r/10:
            x = (l+1)*pr
            cor_X = dx*x + turn_radius*(math.tan(2*x*math.pi+math.pi/2)/2+1)*delx/lon
            cor_Y = dy*x + turn_radius*(math.tan(2*x*math.pi+math.pi/2)/2+1)*dely/lon
            end_side_tp = es
            new_coord_x = coord_x + cor_X
            new_coord_y = coord_y + cor_Y
            position["id"].append(ori["id"].values[0] + l)
            position["id_rg"].append(None)
            position["end_side"].append(end_side_tp)
            position["geometry"].append(Point(new_coord_x, new_coord_y))
            position["Zmsl"].append(numpy.mean([ori["Zmsl"].values[0],
                                                des["Zmsl"].values[0]]))"""
        if l < r/2:
            x = (l+1)*pr
            cor_X = dx*x + turn_radius*(math.tan(2*x*0.6*math.pi+math.pi*0.7)/2+0.5)*delx/lon
            cor_Y = dy*x + turn_radius*(math.tan(2*x*0.6*math.pi+math.pi*0.7)/2+0.5)*dely/lon
            end_side_tp = es
            new_coord_x = coord_x + cor_X
            new_coord_y = coord_y + cor_Y
            if LineString([Point(new_coord_x, new_coord_y), Point(coord_x, coord_y)]).length > 10:
                continue
#            position["id"].append(ori["id"].values[0] + l)
            position["id_rg"].append(None)
            position["end_side"].append(end_side_tp)
            position["geometry"].append(Point(new_coord_x, new_coord_y))
            position["Zmsl"].append(numpy.mean([ori["Zmsl"].values[0],
                                                des["Zmsl"].values[0]]))
        elif l > r/2:
            x = (l)*pr
            cor_X = dx*x + turn_radius*(-math.tan(2*x*0.9*math.pi+math.pi/2)/2+0.5)*delx/lon
            cor_Y = dy*x + turn_radius*(-math.tan(2*x*0.9*math.pi+math.pi/2)/2+0.5)*dely/lon
            end_side_tp = es
            new_coord_x = coord_x + cor_X
            new_coord_y = coord_y + cor_Y
            if LineString([Point(new_coord_x, new_coord_y), Point(coord_x, coord_y)]).length > 10:
                continue
#            position["id"].append(ori["id"].values[0] + l)
            position["id_rg"].append(None)
            position["end_side"].append(end_side_tp)
            position["geometry"].append(Point(new_coord_x, new_coord_y))
            position["Zmsl"].append(numpy.mean([ori["Zmsl"].values[0],
                                                des["Zmsl"].values[0]]))
        """elif l > 9*r/10:
            x = (l+1)*pr
            cor_X = dx*x + turn_radius*(-math.tan(2*x*math.pi+math.pi/2)/2+1)*delx/lon
            cor_Y = dy*x + turn_radius*(-math.tan(2*x*math.pi+math.pi/2)/2+1)*dely/lon
            end_side_tp = es
            new_coord_x = coord_x + cor_X
            new_coord_y = coord_y + cor_Y
            position["id"].append(ori["id"].values[0] + l)
            position["id_rg"].append(None)
            position["end_side"].append(end_side_tp)
            position["geometry"].append(Point(new_coord_x, new_coord_y))
            position["Zmsl"].append(numpy.mean([ori["Zmsl"].values[0],
                                                des["Zmsl"].values[0]]))"""
    # Safety extend end if requested:
    if des.Comment.values[0] == "switched":
        coord_x = des.geometry.x.values[0]
        coord_y = des.geometry.y.values[0]
    else:
        for f in range(3 + ajust_rank, 0, -1):
            # The coord of departur point
            coord_x = des.geometry.x.values[0]+f*.5*delx/(lon)
            coord_y = des.geometry.y.values[0]+f*.5*dely/(lon)
            # Append the safety points point
#            position["id"].append(des["id"].values[0])
            position["id_rg"].append(des["id_rg"].values[0])
            position["end_side"].append(des["end_side"].values[0])
            position["geometry"].append(Point(coord_x, coord_y))
            position["Zmsl"].append(des["Zmsl"].values[0])

    puntos_infernos = gpd.geodataframe.GeoDataFrame(position,
                                                    geometry="geometry",
                                                    crs=crs)
    return puntos_infernos