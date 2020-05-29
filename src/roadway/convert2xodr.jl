function get_lane_length(lane)
    ind = curveindex_end(lane.curve)
    s = lane.curve[ind].s    
end

function get_paramPoly3_coeffs!(lane)
    ox, oy = lane.curve[1].pos.x, lane.curve[1].pos.y
    theta = -mod2pi(lane.curve[1].pos.θ)
    
    new_coods_x, new_coods_y = [], []
    for k = 1:size(lane.curve,1)
        px, py = lane.curve[k].pos.x, lane.curve[k].pos.y        
        px_new = cos(theta) * (px-ox) - sin(theta) * (py-oy)
        py_new = sin(theta) * (px-ox) + cos(theta) * (py-oy)
        append!(new_coods_x, px_new)
        append!(new_coods_y, py_new)
    end
    new_coods_x = Array{Float64}(new_coods_x)
    new_coods_y = Array{Float64}(new_coods_y)

    u = range(0, 1, length=length(new_coods_x))
    poly_x = fit(u, new_coods_x, 3)
    poly_y = fit(u, new_coods_y, 3)
    return poly_x, poly_y
end

function convert_JuliaLaneId2VTDLaneId(laneTag,roadway)
    seg = roadway.segments[laneTag.segment]
    id_Julia = laneTag.lane
    lane_num = length(seg.lanes)
    id_VTD = -(lane_num-id_Julia)-1
    return id_VTD
end

function initialize_XML()
    doc = XMLDocument()
    r=ElementNode("OpenDRIVE")
    setroot!(doc,r)
    header = addelement!(r,"header","")
    header["revMajor"]=1
    header["revMinor"]=4
    header["name"]=""
    header["version"]="1.00"
    header["data"]= "Tue Mar 21 15:00:43 2017"
    header["north"]=0
    header["south"]=0
    header["east"]=0
    header["west"]=0
    return doc,r
end

function add_road_mark!(lane,markType="broken")
    roadMark=addelement!(lane,"roadMark")
    roadMark["sOffset"]=0.0
    roadMark["type"]=markType
    roadMark["weight"]="standard"
    roadMark["color"]="standard"
    roadMark["width"]=0.5
    roadMark["laneChange"]="both"
    return roadMark
end

function convert_lane!(laneTag,right,roadway)
    laneJulia = roadway.segments[laneTag.segment].lanes[laneTag.lane]
    laneWidth = laneJulia.width   
    id = convert_JuliaLaneId2VTDLaneId(laneTag,roadway)
    lane=addelement!(right,"lane")
    lane["id"]=id
    lane["type"]="driving"
    lane["level"]="false"
    width=addelement!(lane,"width")
    width["sOffset"]=0
    width["a"]=laneWidth
    width["b"]=0.0
    width["c"]=0.0
    width["d"]=0.0
    roadMark=add_road_mark!(lane)
    link=addelement!(lane,"link","")
    if !isempty(laneJulia.entrances)
        prevId = convert_JuliaLaneId2VTDLaneId(laneJulia.entrances[1].target.tag,roadway)
        predecessor=addelement!(link,"predecessor")
        predecessor["id"]=prevId
    end
    if !isempty(laneJulia.exits)
        nextId = convert_JuliaLaneId2VTDLaneId(laneJulia.exits[1].target.tag,roadway)
        successor=addelement!(link,"successor")
        successor["id"]=nextId
    end
end

function handle_road_connection!(seg,road,r,roadway) 
    link = addelement!(road,"link","")
    road["junction"] = -1
    lane = seg.lanes[1]
    if !isempty(lane.entrances)
        prevSegId = lane.entrances[1].target.tag.segment
        predecessor = addelement!(link,"predecessor")
        predecessor["elementType"]="road"
        predecessor["elementId"]=prevSegId
        predecessor["contactPoint"]="end"
    end
    if !isempty(lane.exits)
        nextSegId = lane.exits[1].target.tag.segment
        successor = addelement!(link,"successor")
        successor["elementType"]="road"
        successor["elementId"]=nextSegId
        successor["contactPoint"]="start"
    end
end

function convert_seg!(seg,r,roadway)
    road = addelement!(r,"road")
    road["name"]=""
    road["id"]=seg.id
    
    lane=seg.lanes[1]
    s = get_lane_length(lane)
    road["length"]= s
    thetype = addelement!(road,"type")
    thetype["s"]=0
    thetype["type"]="rural"
 
    handle_road_connection!(seg,road,r,roadway)
    
    #deal with planView
    planView=addelement!(road,"planView")
    width = lane.width
    geometry = addelement!(planView,"geometry")
    geometry["s"]=0.0
    geometry["x"]=lane.curve[1].pos.x
    geometry["y"]=lane.curve[1].pos.y
    geometry["hdg"]= mod2pi(lane.curve[1].pos.θ)
    geometry["length"]=s
    
    if lane.curve[1].k == 0 || isnan(lane.curve[1].k)
        print("\nseg", seg.id, " planView : line")
        line=addelement!(geometry,"line")
        
#     else
#         print("\nseg", seg.id, " planView : arc")
#         arc=addelement!(geometry,"arc")
#         arc["curvature"]=lane.curve[1].k

    else
        print("\nseg", seg.id, " planView : paramPoly3")
        poly_x, poly_y = get_paramPoly3_coeffs!(lane)
        paramPoly3=addelement!(geometry,"paramPoly3")
        
        paramPoly3["aU"] = poly_x[0] 
        paramPoly3["bU"] = poly_x[1] 
        paramPoly3["cU"] = poly_x[2]
        paramPoly3["dU"] = poly_x[3] 
        paramPoly3["aV"] = poly_y[0]
        paramPoly3["bV"] = poly_y[1] 
        paramPoly3["cV"] = poly_y[2] 
        paramPoly3["dV"] = poly_y[3]
        paramPoly3["pRange"] = [0,1]   
    end
    
    #deal with lanes
    lanes=addelement!(road,"lanes")
    laneOffset=addelement!(lanes,"laneOffset")
    a= 0.0
    for i = 1:length(seg.lanes)
        a += seg.lanes[i].width
    end
    a -= seg.lanes[1].width/2
    laneOffset["s"]=0.0
    laneOffset["a"]=a
    laneOffset["b"]=0
    laneOffset["c"]=0
    laneOffset["d"]=0
    laneSection=addelement!(lanes,"laneSection")
    laneSection["s"]=0
    left=addelement!(laneSection,"left")
    center=addelement!(laneSection,"center")
    right=addelement!(laneSection,"right")
    
    lane=addelement!(center,"lane","")
    lane["id"]=0
    lane["type"]="driving"
    lane["level"]="false"
    add_road_mark!(lane,"solid")
    
    for lane in seg.lanes
        convert_lane!(lane.tag,right,roadway)
    end  
end

function handle_junction!(r,id,junction,roadway)
    junctionId = id
    junctionVTD = addelement!(r,"junction")
    junctionVTD["name"] = ""
    junctionVTD["id"] = junctionId
    connectionID = 0
    for connection in junction.connections
        # Get an attribute value by name.
        connectionVTD = addelement!(junctionVTD,"connection")
        connectionVTD["id"] = connectionID
        connectionID += 1
        connectionVTD["incomingRoad"] = connection.source
        connectionVTD["connectingRoad"] = connection.path
        connectionVTD["contactPoint"] = "start"
        for i = 1:length(connection.laneConnections)
            laneLink = addelement!(connectionVTD,"laneLink")
            laneLink["from"] = convert_JuliaLaneId2VTDLaneId(LaneTag(connection.source,connection.laneConnections[i][1]),roadway) 
            laneLink["to"] = convert_JuliaLaneId2VTDLaneId(LaneTag(connection.path,i),roadway)
        end
        for road in findall("road", r)
            if parse(Int,road["id"]) == connection.source
                for successor in findall("link/successor", road)
                    successor["elementType"] = "junction"
                    successor["elementId"] = junctionId
                    delete!(successor, "contactPoint")
                end
                for successor in findall("lanes/laneSection/right/lane/link/successor", road)
                    unlink!(successor)
                end
            end
            if parse(Int,road["id"]) == connection.path
                road["junction"] = junctionId
                for roadi in findall("road", r)
                    if parse(Int,roadi["id"]) == roadway.segments[connection.path].lanes[1].exits[1].target.tag.segment
                        for predecessor in findall("link/predecessor", roadi)
                            predecessor["elementType"] = "junction"
                            predecessor["elementId"] = junctionId
                            delete!(predecessor, "contactPoint")
                        end
                        for predecessor in findall("lanes/laneSection/right/lane/link/predecessor", roadi)
                            unlink!(predecessor)
                        end  
                    end     
                end
            end
        end
    end
end

function handle_junctions(r,junctions,roadway)
    for i = 1:length(junctions)
        handle_junction!(r,i,junctions[i],roadway)
    end
end

function convert_roadway!(r,roadway)
    for seg in roadway.segments
        convert_seg!(seg,r,roadway)
    end
    #resolveRoadJunction!(r,roadway)
end
