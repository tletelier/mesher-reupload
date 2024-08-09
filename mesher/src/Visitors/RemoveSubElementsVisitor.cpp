/*
 <Mix-mesher: region type. This program generates a mixed-elements mesh>
 
 Copyright (C) <2013,2017>  <Claudio Lobos>
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/gpl.txt>
 */

#include "RemoveSubElementsVisitor.h"
#include "../Octant.h"

namespace Clobscode
{

    RemoveSubElementsVisitor::RemoveSubElementsVisitor():points(NULL)
    { }

    void RemoveSubElementsVisitor::setPoints(vector<MeshPoint> &points) {
        this->points = &points;
    }
    void RemoveSubElementsVisitor::setTriMesh(TriMesh &mesh) {
        this->mesh = &mesh;
    }
    void RemoveSubElementsVisitor::setFaces(list<unsigned int> &faces){
        this->faces = &faces;
    }
    

    bool RemoveSubElementsVisitor::visit(Octant *o) { 
        vector<vector<unsigned int>> &sub_elements = o->sub_elements;

        list<unsigned int> faces_inter = o->intersected_faces;
        list<vector<unsigned int> > still_in;
        list<vector<unsigned int> > to_review;
        list<vector<unsigned int> >::iterator iter;
        list<vector<unsigned int> >::iterator it;
        IntersectionsVisitor iv; // visitors
        vector<unsigned int> &pointindex = o->pointindex;
        double max_dis = o->getMaxDistance();
        for (unsigned int i=0; i<sub_elements.size(); i++) {
            bool onein = false;
            vector<unsigned int> e_pts = sub_elements[i];
            unsigned int cant = 0;
            for (unsigned int j=0; j<e_pts.size(); j++) {
                if (points->at(e_pts[j]).isInside() && !points->at(e_pts[j]).wasProjected()) {  //Notes: Verifica si el punto esta adentro
                    onein = true;
                    cant++;
                }
            }
            if (onein) {
                o->setSurface();
                return false;
                still_in.push_back(sub_elements[i]);
            }
            else {
                Point3D avg;
                // Calculates centroid
                for (unsigned int i =0; i<e_pts.size(); i++){
                    avg += points->at(o->getPoints()[i]).getPoint(); 
                }
                avg /= e_pts.size();

                if (mesh->pointIsInMesh(avg, faces_inter)){
                    o->setSurface();
                    for (unsigned int j=0; j<e_pts.size(); j++) { 
                        if (points->at(e_pts[j]).wasProjected()){
                            points->at(e_pts[j]).setInside();
                        }
                    }
                    return false;
                } 

                if (cant == 0) {  
                    for (unsigned int j=0; j<e_pts.size(); j++) {
                        if (points->at(e_pts[j]).wasProjected()){
                            // Calculates new point moved towards centroide
                            Point3D newP = newPointTowardsCentroide(points->at(e_pts[j]).getPoint(), max_dis, avg);
                            if (mesh->pointIsInMesh(newP, faces_inter)){
                                points->at(e_pts[j]).setInside();
                                o->setSurface();
                                return false;
                            }
                        }
                    }
                }
            }
        }

        if (still_in.size()==sub_elements.size()) { 
            //Notes: False if there's still elements inside.
            return false;
        }
        if (still_in.empty()) {
            return true;

        }

        sub_elements.clear();
        sub_elements.reserve(still_in.size());
        for (iter=still_in.begin(); iter!=still_in.end(); iter++) {
            sub_elements.push_back(*iter);
        }
        return false;
    }
    bool RemoveSubElementsVisitor::edgeTriangleIntersection(SurfTriangle &st,
                                                   vector<Point3D> &input_pts,
                                                   vector<vector<Point3D>> &oct_edges) {

        //test each edge against the triangle
        for (unsigned int i=0; i<oct_edges.size(); i++) {
            vector<Point3D> oct_ed = oct_edges[i];
            if (st.segmentIntersection(input_pts,oct_ed[0],oct_ed[1])) {
                return true;
            }
        }

        return false;
    }

    Point3D RemoveSubElementsVisitor::newPoint(Point3D &point, double &dis) {
        Point3D newPoint;
        //Move the point in a distance dis.
        for (unsigned int i=0; i<3; i++) {
            newPoint[i] = point[i] + dis;
        }
        return newPoint;
    }


    Point3D RemoveSubElementsVisitor::newPointTowardsCentroide(Point3D &point, double &targetDistance, Point3D &centroid) {
        // Check for invalid distance
        if (targetDistance <= 0) {
            throw std::invalid_argument("Target distance must be positive.");
        }
        // Calculate the direction vector from 'point' to 'centroid'
        Point3D direction = centroid - point;
        // Normalize the direction vector
        // Move the point in the direction by the desired distance
        Point3D newPoint = point + direction.normalize() * targetDistance;
        return newPoint;
    }
}