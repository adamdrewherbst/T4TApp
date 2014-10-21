#include "T4TApp.h"
#include "CGAL/Simple_cartesian.h"
#include "CGAL/Polyhedron_incremental_builder_3.h"
#include "CGAL/Polyhedron_3.h"
#include "CGAL/Nef_polyhedron_3.h"
#include "CGAL/basic.h"
#include "CGAL/iterator.h"

template <class HDS>
class Node2Poly : public CGAL::Modifier_base<HDS> {
public:
	MyNode *_node;
	
    Node2Poly(MyNode *node) {
    	_node = node;
    }
    
    void operator()( HDS& hds) {
        // Postcondition: hds is a valid polyhedral surface.
        CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
        short nv = _node->nv(), nf = _node->nf(), ne = _node->ne(), i, j, n;
        Vector3 v;
        B.begin_surface(nv, nf, 2*ne);
        typedef typename HDS::Vertex   Vertex;
        typedef typename Vertex::Point Point;
        for(i = 0; i < nv; i++) {
        	v = _node->_worldVertices[i];
        	B.add_vertex(Point(v.x, v.y, v.z));
        	cout << "adding vertex " << v.x << "," << v.y << "," << v.z << endl;
        }
        for(i = 0; i < nf; i++) {
        	n = _node->_faces[i].size();
        	B.begin_facet();
        	cout << "adding facet: ";
        	for(j = 0; j < n; j++) {
        		B.add_vertex_to_facet(_node->_faces[i][j]);
        		cout << _node->_faces[i][j] << "\t";
        	}
        	cout << endl;
        	B.end_facet();
        }
        B.end_surface();
    }
};

typedef CGAL::Simple_cartesian<float>      Kernel;
typedef CGAL::Vector_3<Kernel>             Vector;
typedef CGAL::Point_3<Kernel>              Point;
typedef CGAL::Plane_3<Kernel>              Plane_3;
typedef CGAL::Polyhedron_3<Kernel>         Polyhedron;
typedef Polyhedron::HalfedgeDS             HalfedgeDS;
typedef Polyhedron::Vertex_const_iterator  VCI;
typedef Polyhedron::Facet_const_iterator   FCI;
typedef Polyhedron::Halfedge_around_facet_const_circulator HFCC;
typedef CGAL::Nef_polyhedron_3<Kernel>     Nef;
typedef CGAL::Inverse_index<VCI> Index;

Nef nodeToNef(MyNode *node) {
    Polyhedron poly;
    Node2Poly<HalfedgeDS> converter(node);
    poly.delegate(converter);
    return Nef(poly);
}

bool ToolMode::drillCGAL() {
	//convert the node mesh to a Nef polyhedron
	Polyhedron nodePoly, toolPoly;
	Node2Poly<HalfedgeDS> nodeConv(_node), toolConv(_tool);
	nodePoly.delegate(nodeConv);
	toolPoly.delegate(toolConv);
	Nef nodeNef(nodePoly);
	Nef toolNef(toolPoly);
	//make another Nef polyhedron from the drillbit side planes
/*	Tool *tool = getTool();
	short segments = tool->iparam[0], i, j;
	float radius = tool->fparam[0], angle, dAngle = 2*M_PI / segments, planeDistance = radius * cos(dAngle/2), f1;
	Vector3 v1, v2;
	Matrix toolWorld = _tool->getWorldMatrix();
	Nef toolNef;
	planes.resize(segments+2);
	for(i = 0; i < segments; i++) {
		angle = (2*M_PI*i) / segments;
		//plane
		planes[i].setNormal(cos(angle+dAngle/2), sin(angle+dAngle/2), 0.0f);
		planes[i].setDistance(-planeDistance);
	}
	for(i = 0; i < 2; i++) {
		planes[segments+i].setNormal(0, 0, 1.0f*(2*i-1));
		planes[segments+i].setDistance(-100.0f);
	}
	Vector3 toolCenter = _tool->getTranslationWorld();
	for(i = 0; i < segments+2; i++) {
		planes[i].transform(toolWorld);
		v1 = planes[i].getNormal();
		f1 = planes[i].getDistance();
		//make sure the plane normal points outward from the drill center
		if(v1.dot(-v1*f1 - toolCenter) < 0) {
			planes[i].set(-v1, -f1);
		}
		v1 = -planes[i].getNormal() * planes[i].getDistance();
		v2 = -planes[i].getNormal();
		Plane_3 plane(Point(v1.x, v1.y, v1.z), Vector(v2.x, v2.y, v2.z));
		toolNef += Nef(plane);
	}*/
	//Nef-subtract the drill from the node
//	nodeNef -= toolNef;
	//convert from Nef back to Polyhedron
	Polyhedron newPoly;
	nodeNef.convert_to_polyhedron(newPoly);
	//extract the new mesh data from the result
	for(VCI v = newPoly.vertices_begin(); v != newPoly.vertices_end(); v++) {
		_newNode->addVertex((float)(v->point().x()), (float)(v->point().y()), (float)(v->point().z()));
	}
	Index index(newPoly.vertices_begin(), newPoly.vertices_end());
	std::vector<unsigned short> face;
	for(FCI f = newPoly.facets_begin(); f != newPoly.facets_end(); f++) {
		HFCC h = f->facet_begin(), he = h;
		std::size_t n = circulator_size(h), i = 0;
		face.resize(n);
		do {
			face[i++] = index[VCI(h->vertex())];
			++h;
		} while(h != he);
		_newNode->addFace(face);
	}
	_newNode->_hulls = _node->_hulls;
	return true;
}


