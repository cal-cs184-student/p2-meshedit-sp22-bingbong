#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
    std::vector<Vector2D> decast = std::vector<Vector2D>();
      for (int i = 0; i < points.size() - 1; i++) {
          decast.push_back((1-t)*points[i] + (t)*points[i+1]);
      }
      return decast;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      std::vector<Vector3D> decast = std::vector<Vector3D>();
        for (int i = 0; i < points.size() - 1; i++) {
            decast.push_back((1-t)*points[i] + (t)*points[i+1]);
        }
        return decast;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      if (points.size() <= 1) {
          return points[0];
      }
      
      std::vector<Vector3D> points2;
      int i = 0;
      while (i == 0 || points2.size() > 1) {
          if (i == 0) {
              points2 = evaluateStep(points, t);
          }
          else {
              points2 = evaluateStep(points2, t);
          }
          i++;
      }
      
      return points2[0];
      
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    std::vector<Vector3D> other_axis = std::vector<Vector3D>();
      for (int i = 0; i < controlPoints.size(); i++) {
          other_axis.push_back(evaluate1D(controlPoints[i], u));
      }
    return evaluate1D(other_axis, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
      HalfedgeCIter h = halfedge();      // get the outgoing half-edge of the vertex
      HalfedgeCIter start = halfedge();
      Vector3D sum = Vector3D();
      do {
          VertexCIter v1 = h->vertex();
          h = h->next();
          VertexCIter v2 = h->vertex();
          h = h->next();
          VertexCIter v3 = h->vertex();
          Vector3D cross_product = cross(v2->position - v1->position, v3->position - v1->position) / 2;

          sum += cross_product;
          h = h->twin();
      } while (h != start);
    //normalize HERE
    return sum / sqrt(pow(sum.x, 2) + pow(sum.y, 2) + pow(sum.z, 2));
  }
  
  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.

      if (e0->halfedge()->isBoundary() || e0->halfedge()->twin()->isBoundary()) {
          return e0;
    }
    //save all the old stuff
      HalfedgeIter h = e0->halfedge();
      //old halfedges
      HalfedgeIter ha = h;
      HalfedgeIter hta = h->twin();

      HalfedgeIter hb = h->next();
      HalfedgeIter hbt = h->next()->twin();

      HalfedgeIter hc = h->next()->next();
      HalfedgeIter hct = h->next()->next()->twin();

      HalfedgeIter htb = hta->next();
      HalfedgeIter htbt = hta->next()->twin();

      HalfedgeIter htc = hta->next()->next();
      HalfedgeIter htct = hta->next()->next()->twin();
      //old edges
      EdgeIter hea = e0;
      EdgeIter heb = hb->edge();
      EdgeIter hec = hc->edge();
      EdgeIter hteb = htb->edge();
      EdgeIter htec = htc->edge();
      //old vertices
      VertexIter a = h->next()->next()->vertex();
      VertexIter b = h->vertex();
      VertexIter c = h->next()->vertex();
      VertexIter d = h->twin()->next()->next()->vertex();
      //old faces
      FaceIter fh = h->face();
      FaceIter fht = h->twin()->face();
      
    //DO THE NEW STUFF
    //For each vertex, edge, and face, set its halfedge pointer.
      //vertices
      a->halfedge() = hc;
      b->halfedge() = htb;
      c->halfedge() = hb;
      d->halfedge() = htc;
      //halfedges
      //sN(next, twin, vertex, edge, face)
      (*ha).setNeighbors(htc, hta, a, hea, fh);
      (*htc).setNeighbors(hb, htct, d, htec, fh);
      (*hb).setNeighbors(ha, hbt, c, heb, fh);

      (*hta).setNeighbors(hc, ha, d, hea, fht);
      (*hc).setNeighbors(htb, hct, a, hec, fht);
      (*htb).setNeighbors(hta, htbt, b, hteb, fht);
      //edge
      heb->halfedge() = hb;
      hec->halfedge() = hc;
      hea->halfedge() = ha;
      hteb->halfedge() = htb;
      htec->halfedge() = htc;
      //face
      fh->halfedge() = ha;
      fht->halfedge() = hta;
      
    return EdgeIter();
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
          //save all the old stuff
      if (e0->halfedge()->isBoundary() || e0->halfedge()->twin()->isBoundary()) {
          return e0->halfedge()->vertex();
      }
      HalfedgeIter h = e0->halfedge();
      //old halfedges
      HalfedgeIter ha = h;
      HalfedgeIter hta = h->twin();

      HalfedgeIter hb = h->next();
      HalfedgeIter hbt = h->next()->twin();

      HalfedgeIter hc = h->next()->next();
      HalfedgeIter hct = h->next()->next()->twin();

      HalfedgeIter htb = hta->next();
      HalfedgeIter htbt = hta->next()->twin();

      HalfedgeIter htc = hta->next()->next();
      HalfedgeIter htct = hta->next()->next()->twin();
      //old edges
      EdgeIter hea = e0;
      hea->isNew = false;
      hea->isNew2 = false;
      EdgeIter heb = hb->edge();
      EdgeIter hec = hc->edge();
      EdgeIter hteb = htb->edge();
      EdgeIter htec = htc->edge();
      //old vertices
      VertexIter a = h->next()->next()->vertex();
      VertexIter b = h->vertex();
      VertexIter c = h->next()->vertex();
      VertexIter d = h->twin()->next()->next()->vertex();
      //old faces
      FaceIter fh = h->face();
      FaceIter fht = h->twin()->face();

      //make the new stuff
      VertexIter nv = newVertex();
      nv->isNew = true;
      FaceIter nfa = newFace();
      FaceIter nfb = newFace();
      EdgeIter nea = newEdge();
      nea->isNew = true;
      nea->isNew2 = true;
      EdgeIter neb = newEdge();
      neb->isNew = false; //this is the bottom half of the old edge, so its not really a new edge (its black not blue in loop subdivisoin)
      neb->isNew2 = true; //note that this isnt actually a new edge (its half of the old edge), but we set it as new so we dont infinite loop in loop subdivison. can't trust the isNew param for edges.
      EdgeIter nec = newEdge();
      nec->isNew = true;
      nec->isNew2 = true;
      HalfedgeIter nha = newHalfedge();
      HalfedgeIter nhat = newHalfedge();
      HalfedgeIter nhb = newHalfedge();
      HalfedgeIter nhbt = newHalfedge();
      HalfedgeIter nhc = newHalfedge();
      HalfedgeIter nhct = newHalfedge();

      //set new vertex
      Vector3D newpos = Vector3D();
      newpos.x = (c->position.x + b->position.x) / 2.0;
      newpos.y = (c->position.y + b->position.y) / 2.0;
      newpos.z = (c->position.z + b->position.z) / 2.0;
      nv->position = newpos;
      nv->halfedge() = ha;

      //set everythinggg


      //sN(next, twin, vertex, edge, face
      (*ha).setNeighbors(hb, hta, nv, hea, fh);
      (*hb).setNeighbors(nha, hbt, c, heb, fh);
      (*nha).setNeighbors(ha, nhat, a, nea, fh);

      (*nhat).setNeighbors(hc, nha, nv, nea, nfa);
      (*hc).setNeighbors(nhb, hct, a, hec, nfa);
      (*nhb).setNeighbors(nhat, nhbt, b, neb, nfa);

      (*nhbt).setNeighbors(htb, nhb, nv, neb, nfb);
      (*htb).setNeighbors(nhc, htbt, b, hteb, nfb);
      (*nhc).setNeighbors(nhbt, nhct, d, nec, nfb);

      (*nhct).setNeighbors(htc, nhc, nv, nec, fht);
      (*htc).setNeighbors(hta, htct, d, htec, fht);
      (*hta).setNeighbors(nhct, ha, c, hea, fht);

      //faces
      fh->halfedge() = ha;
      fht->halfedge() = hta;
      nfa->halfedge() = nhb;
      nfb->halfedge() = nhbt;

      // edges
      hea->halfedge() = ha;
      nea->halfedge() = nha;
      neb->halfedge() = nhb;
      nec->halfedge() = nhc;
      // vertices
      c->halfedge() = hb;
      a->halfedge() = nha;
      b->halfedge() = nhb;
      d->halfedge() = nhc;

    return nv;
  }

void Vertex::computeCentroid() {
    Vector3D avg = Vector3D(); // avg is actually a sum, not avg
    HalfedgeIter h = halfedge();      // get the outgoing half-edge of the vertex
        do {
            HalfedgeIter h_twin = h->twin(); // get the opposite half-edge
            VertexIter v2 = h_twin->vertex(); // vertex is the 'source' of the half-edge, so
            avg += v2->position;
            // h->vertex() is v, whereas h_twin->vertex()
                                              // is the neighboring vertex
            h = h_twin->next();               // move to the next outgoing half-edge of the vertex
        } while(h != halfedge());
    centroid = avg;
}



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
      
      for( VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++ )
          {
              if (v->isNew) {
                  cout << "Bad";
              }
              v->isNew = false;
              Vector3D og_pos = Vector3D(v->position);
              int num_neighbors = v->degree();
              
              v->computeCentroid();
              Vector3D sum_neighbors = v->centroid;
              
              float u;
              
            if (num_neighbors == 3) {
                  u = 3.0/16.0;
              } else {
                  u = 3.0/(8.0*num_neighbors);
              }
              
              v->newPosition = (1.0-num_neighbors*u)*og_pos + u*sum_neighbors;
          }
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
      
      for( EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++ )
          {
              Vector3D p0 = e->halfedge()->vertex()->position;
              Vector3D p1 = e->halfedge()->twin()->vertex()->position;
              Vector3D p2 = e->halfedge()->next()->next()->vertex()->position;
              Vector3D p3 = e->halfedge()->twin()->next()->next()->vertex()->position;
              Vector3D new_pos = (3.0/8) * (p0 + p1) + (1.0/8)*(p2 + p3);
              
              e->newPosition = new_pos;
              e->isNew = false;
              e->isNew2 = false;
          }
      
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
      
      for( EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++ )
          {
              if (e->isNew == false && e->isNew2 == false) {
                  VertexIter new_v = mesh.splitEdge(e);
                  new_v->newPosition = e->newPosition;
              }
          }
      
      
    // 4. Flip any new edge that connects an old and new vertex.
      for( EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++ )
          {
              if (e->isNew == true) {
                  if ((e->halfedge()->vertex()->isNew==true && e->halfedge()->twin()->vertex()->isNew == false) ||
                      (e->halfedge()->vertex()->isNew==false && e->halfedge()->twin()->vertex()->isNew == true)) {
                      mesh.flipEdge(e);
                  }
              }
          }

    // 5. Copy the new vertex positions into final Vertex::position.
      for( VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++ )
          {
              if (v->isNew != true) {
                  v->position = v->newPosition;
              } else {
                  v->isNew = false;
                  v->position = v->newPosition;
              }
          }
      for( EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++ )
          {
              if (e->isNew == false && e->isNew2==false) {
                  int abc = 0;
                 //e->halfedge()->next()->vertex()->position = e->newPosition;
                 //e->halfedge()->vertex()->position = e->newPosition;
              } else {
                  e->isNew = false;
                  e->isNew2 = false;
              } 
            
          }
  }
}
