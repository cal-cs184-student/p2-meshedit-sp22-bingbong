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
      b->halfedge() = ha;
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
      htec->halfedge() = htb;
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
    return VertexIter();
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    
    // 4. Flip any new edge that connects an old and new vertex.

    // 5. Copy the new vertex positions into final Vertex::position.

  }
}
