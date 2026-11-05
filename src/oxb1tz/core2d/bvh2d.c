#include "oxb1tz/core2d/bvh2d.h"
#include "oxb1tz/core2d/intersect2d.h"
#include "oxb1tz/memory.h"
#include "oxb1tz/math.h"

/**********************************************
*               AABB OPERATION                *
***********************************************/


oxb_inline oxb_aabb2d combine(const oxb_aabb2d a, const oxb_aabb2d b) {
 oxb_aabb2d out;
 oxb_vec2 min_val_a = oxb_vec2_load(a._min._arr);
 oxb_vec2 min_val_b = oxb_vec2_load(b._min._arr);

 oxb_vec2 max_val_a = oxb_vec2_load(a._max._arr);
 oxb_vec2 max_val_b = oxb_vec2_load(b._max._arr);
 
 oxb_vec2_store(out._min._arr, oxb_vec2_min(min_val_a, min_val_b));
 oxb_vec2_store(out._max._arr, oxb_vec2_max(max_val_a, max_val_b));
 return out;
}


oxb_inline bool overlaps(const oxb_aabb2d a, const oxb_aabb2d b) {
 return a._min._arr[0] <= b._max._arr[0] &&
   a._max._arr[0] >= b._min._arr[0] &&
   a._min._arr[1] <= b._max._arr[1] &&
   a._max._arr[1] >= b._min._arr[1];
}


oxb_inline float area(const oxb_aabb2d a) {
 float w = a._max._arr[0] - a._min._arr[0];
 float h = a._max._arr[1] - a._min._arr[1];
 return w * h;
}


oxb_inline bool contains(const oxb_aabb2d a, const oxb_aabb2d b) {
 return a._min._arr[0] <= b._min._arr[0] &&
  a._min._arr[1] <= b._min._arr[1] &&
  a._max._arr[0] >= b._max._arr[0] &&
  a._max._arr[1] >= b._max._arr[1];
}

/*
oxb_inline bool nearly_equal(const oxb_aabb2d a, const oxb_aabb2d b) {
 oxb_vec2 min_a = oxb_vec2_load(a._min._arr);
 oxb_vec2 max_a = oxb_vec2_load(a._max._arr);

 oxb_vec2 min_b = oxb_vec2_load(b._min._arr);
 oxb_vec2 max_b = oxb_vec2_load(b._max._arr);
 oxb_vec2 extent_a = oxb_vec2_abs(oxb_vec2_sub(min_a, min_b));
 oxb_vec2 extent_b = oxb_vec2_abs(oxb_vec2_sub(max_a, max_b));
 const float epsilon = 1e-16f;
 return oxb_vec2_getx(extent_a) < epsilon && oxb_vec2_gety(extent_a) < epsilon && oxb_vec2_getx(extent_b) < epsilon && oxb_vec2_gety(extent_b) < epsilon;
}
*/


oxb_inline oxb_aabb2d expanded(const oxb_aabb2d a, float amount) {
 assert(amount > 0.0f);
 oxb_aabb2d out;
 oxb_vec2 vector_amount = oxb_vec2_splat(amount);
 oxb_vec2 min_val_a = oxb_vec2_load(a._min._arr);
 oxb_vec2 max_val_a = oxb_vec2_load(a._max._arr);
 
 min_val_a = oxb_vec2_sub(min_val_a, vector_amount);
 max_val_a = oxb_vec2_add(max_val_a, vector_amount);
 
 oxb_vec2_store(out._min._arr, min_val_a);
 oxb_vec2_store(out._max._arr, max_val_a);
 return out;
}


/**********************************************
*                   BVH TREE                  *
***********************************************/


oxb_inline bool is_leaf(const oxb_bvh2d_node n) {
 return oxb_pool_id_isnull(n._left) && oxb_pool_id_isnull(n._right);
}



/*
       • --> _root
      / \
     /   \
    •      • ----> current_node
   / \    / \
  /   \  /   \
 •     ••     •
        |     |---> _right 
        |
        -------> _left
 
 combine left and right, and result would be their parent's aabb
 repeat until they reach the root.
 
 it process from current node, up to the root.
*/
oxb_noinline static void refit_upwards(oxb_bvh2d *const bvh, oxb_pool_id n) {
 assert(!oxb_pool_id_isnull(n));
 oxb_pool_id node = n;
 while(!oxb_pool_id_isnull(node)) {  
  oxb_bvh2d_node *node_ptr = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, node);
  if(!is_leaf(*node_ptr)) {
   oxb_bvh2d_node *left = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, node_ptr->_left);
   oxb_bvh2d_node *right = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, node_ptr->_right);
   
   oxb_aabb2d combined = combine(left->_aabb, right->_aabb);


   if(combined._min._arr[0] == node_ptr->_aabb._min._arr[0]
   && combined._min._arr[1] == node_ptr->_aabb._min._arr[1]
   && combined._max._arr[0] == node_ptr->_aabb._max._arr[0]
   && combined._max._arr[1] == node_ptr->_aabb._max._arr[1]
   )
    return;
    
   node_ptr->_aabb = combined;
   }
  node = node_ptr->_parent;
 }
}




/*
       • --> _root
      / \
     /   \
    •      • --> calculate cost of left and right of every node
   / \    / \    then compare, and always to go to the lowest cost
  /   \  /   \   to make it at least balanced
 •     ••     • 
              |----> leaf node

 starts from the root and travel all the way down the leaf with minimum
 possible cost.
 
 • if you find the leaf with minimum cost, that would be the best sibling
 
 • the cost means smaller aabb result when you refit it upwards.
 • for farther object, it would result in a larger aabb(higher cost).
*/
/*
oxb_noinline static oxb_pool_id find_best_sibling(oxb_bvh2d *const bvh, oxb_pool_id leaf_id) {
 assert(!oxb_pool_id_isnull(bvh->_root));
  
 oxb_bvh2d_node *leaf = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, leaf_id);
 oxb_pool_id current = bvh->_root;
 oxb_bvh2d_node *current_data = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, current);

 while(!is_leaf(*current_data)) {
  oxb_bvh2d_node *node = current_data;

  oxb_bvh2d_node *left = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, node->_left);
  oxb_bvh2d_node *right = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, node->_right);

  oxb_aabb2d combined = combine(node->_aabb, leaf->_aabb);

  float inheritance_cost = (area(combined) - area(node->_aabb));
  float cost1, cost2;
  if(is_leaf(*left)) {
   oxb_aabb2d box = combine(leaf->_aabb, left->_aabb);
   cost1 = area(box) + inheritance_cost;
  } else {
   oxb_aabb2d box = combine(leaf->_aabb, left->_aabb);

   float old_area = area(left->_aabb);
   float new_area = area(box);
   cost1 = (new_area - old_area) + inheritance_cost;
  }
  if(is_leaf(*right)) {
   oxb_aabb2d box = combine(leaf->_aabb, right->_aabb);
   cost2 = area(box) + inheritance_cost;
  } else {
   oxb_aabb2d box = combine(leaf->_aabb, right->_aabb);

   float old_area = area(right->_aabb);
   float new_area = area(box);
   cost2 = (new_area - old_area) + inheritance_cost;
  }
    
  if(cost1 < cost2) {
   current_data = left;
  } else {
   current_data = right;
  }
  current = current_data->_allocation;
 }
 return current;
}
*/
oxb_noinline static oxb_pool_id find_best_sibling(oxb_bvh2d *const bvh, oxb_pool_id leaf_id) {
 assert(!oxb_pool_id_isnull(bvh->_root));

 oxb_bvh2d_node *leaf = (oxb_bvh2d_node *)oxb_pool_get(&bvh->_pool, leaf_id);
 oxb_pool_id current = bvh->_root;

 do {
  oxb_bvh2d_node *node = (oxb_bvh2d_node *)oxb_pool_get(&bvh->_pool, current);
  
  /* this leaf best sibling? */
  if(is_leaf(*node)) {
   return current;
  }

  oxb_bvh2d_node *left = (oxb_bvh2d_node *)oxb_pool_get(&bvh->_pool, node->_left);
  oxb_bvh2d_node *right = (oxb_bvh2d_node *)oxb_pool_get(&bvh->_pool, node->_right);

  oxb_aabb2d combined = combine(node->_aabb, leaf->_aabb);

  float combined_area = area(combined);
  float node_area = area(node->_aabb);
  float parent_cost = 2.0f * combined_area;
  float inheritance_cost = 2.0f * (combined_area - node_area);

  /* left child cost */
  oxb_aabb2d left_combined = combine(left->_aabb, leaf->_aabb);

  float left_area = area(left->_aabb);
  float left_combined_area = area(left_combined);

  float left_cost;

  if(is_leaf(*left)) {
   left_cost = left_combined_area + inheritance_cost;
  } else {
   left_cost = (left_combined_area - left_area) + inheritance_cost;
  }

  /* right child cost */
  oxb_aabb2d right_combined = combine(right->_aabb, leaf->_aabb);

  float right_area = area(right->_aabb);
  float right_combined_area = area(right_combined);
  
  float right_cost;

  if(is_leaf(*right)) {
   right_cost = right_combined_area + inheritance_cost;
  } else {
   right_cost = (right_combined_area - right_area) + inheritance_cost;
  }

  /* none of them gives a better cost */
  if(parent_cost < left_cost && parent_cost < right_cost) {
   return current;
  }

  /* always choose the cheaper one. */
  if(left_cost < right_cost) {
   current = node->_left;
  } else {
   current = node->_right;
  }
 } while(true);
}



/*
 create an empty leaf node
*/
oxb_noinline static oxb_pool_id create_leaf(oxb_bvh2d *const bvh, const oxb_aabb2d aabb, const oxb_handle body_id) {
 oxb_pool_id leaf_id = oxb_pool_acquire(&bvh->_pool);
 oxb_bvh2d_node *leaf = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, leaf_id);
 leaf->_left = OXB_POOL_NULL_ID;
 leaf->_right = OXB_POOL_NULL_ID;
 leaf->_parent = OXB_POOL_NULL_ID;
 leaf->_allocation = leaf_id;
 leaf->_aabb = aabb;
 leaf->_body = body_id;
 return leaf_id;
}


/*
       • --> _root
      / \
     /   \
    •      • 
   / \    / \ 
  /   \  /   \ 
 •     ••     • 
              |----> (lets say) best sibling (always leaf)

 then the inserted leaf will become like this.

       • --> _root
      / \
     /   \
    •      • 
   / \    / \ 
  /   \  /   \ 
 •     ••     • --> newly alloated parent (non-leaf)
             / \
            /   \
           •     • ----> inserted leaf
           |
           |------->  best sibling
           
 after that, it has to refit upwards up to the root. 
*/

oxb_noinline static void insert_leaf(oxb_bvh2d *const bvh, const oxb_pool_id leaf_id) {
 if(oxb_pool_id_isnull(bvh->_root)) {
  bvh->_root = leaf_id;
  return;
 }

 oxb_pool_id new_parent = OXB_POOL_NULL_ID;
 oxb_bvh2d_node *new_parent_ptr = NULL;
 {
  new_parent = oxb_pool_acquire(&bvh->_pool);
  new_parent_ptr = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, new_parent);
  new_parent_ptr->_left = OXB_POOL_NULL_ID;
  new_parent_ptr->_right = OXB_POOL_NULL_ID;
  new_parent_ptr->_parent = OXB_POOL_NULL_ID;
  new_parent_ptr->_allocation = new_parent;
 }

 /*
  find the best sibling and place the new leaf besides that 
  best sibling
 */
 oxb_bvh2d_node *leaf = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, leaf_id);
 oxb_pool_id sibling = find_best_sibling(bvh, leaf_id);
 oxb_bvh2d_node *sibling_ptr = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, sibling);
 
 oxb_pool_id old_parent = sibling_ptr->_parent;
 oxb_bvh2d_node *old_parent_ptr = NULL;
 
 bool old_parent_valid = !oxb_pool_id_isnull(old_parent);
 if(old_parent_valid) 
  old_parent_ptr = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, old_parent);

 /*
  reconnect the nodes, to make it valid id
 */
 new_parent_ptr->_parent = old_parent;

 new_parent_ptr->_left = sibling;
 new_parent_ptr->_right = leaf_id;

 sibling_ptr->_parent = new_parent;
 leaf->_parent = new_parent;

 if(old_parent_valid) {
  if(oxb_pool_id_isequal(old_parent_ptr->_left, sibling))
   old_parent_ptr->_left = new_parent;
  else
   old_parent_ptr->_right = new_parent;
 } else {
  bvh->_root = new_parent;
 }
 refit_upwards(bvh, new_parent);
}


/*
 disconnect only the leaf_id(no release, only its parent), so it can be reinsert again

       • --> grand parent
      / \
     /   \
    •      •  -> parent
   / \    / \ 
  /   \  /   \ 
 •     ••     • 
              |----> (lets say) leaf that you want to remove

 next step
       • --> grand parent
      / \
     /   \
    •      •  --> release the parent's memory
   / \    / \ 
  /   \  /   \ 
 •     ••     • 
        |     |----> (lets say) leaf that you want to remove
        |
        |------> connect the sibling to the grand parent's child

       • --> grand parent
      / \
     /   \
    •     •  --> previous sibling
   / \ 
  /   \
 •     •

 after the remove, it has to be refited upwards
 to update the aabb size.
*/
oxb_noinline static void remove_leaf(oxb_bvh2d *const bvh, oxb_pool_id leaf_id) {
 if(oxb_pool_id_isequal(leaf_id, bvh->_root)) {
  bvh->_root = OXB_POOL_NULL_ID;
  return;
 }
 oxb_bvh2d_node *leaf = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, leaf_id);
 oxb_pool_id parent = leaf->_parent;
 oxb_bvh2d_node *parent_ptr = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, parent);
 
 assert(oxb_pool_id_isequal(parent_ptr->_left, leaf_id) || oxb_pool_id_isequal(parent_ptr->_right, leaf_id));

 oxb_pool_id grand_parent = parent_ptr->_parent;
 oxb_bvh2d_node *grand_parent_ptr = NULL;
 bool grand_parent_valid = !oxb_pool_id_isnull(parent_ptr->_parent);

 if(grand_parent_valid)
  grand_parent_ptr = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, grand_parent);

 oxb_pool_id sibling = oxb_pool_id_isequal(parent_ptr->_left, leaf_id) ? parent_ptr->_right : parent_ptr->_left;

 if(grand_parent_valid) {
  oxb_bvh2d_node *sibling_ptr = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, sibling);

  if(oxb_pool_id_isequal(grand_parent_ptr->_left, parent))
   grand_parent_ptr->_left = sibling;
  else
   grand_parent_ptr->_right = sibling;
  sibling_ptr->_parent = grand_parent;
  refit_upwards(bvh, grand_parent);
 } else {
  /* the grandparent is root */
  bvh->_root = sibling;
  oxb_bvh2d_node *sibling_ptr = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, sibling);
  sibling_ptr->_parent = OXB_POOL_NULL_ID;
 }
 oxb_pool_release(&bvh->_pool, parent);
}




/*
  pair traversal recursive
  
       • --> _root
      / \
  |--•   \
  |       • -->  right
left     / \ 
        /   \ --> subtree
       •     •
      / \   / \
     /   \ •   \
    •     •     •
 
 if right_id is a leaf
 you use the right_id, compare it against the entire subtree(child_left, child_right)
 
 if left_id is a leaf
 then do the same thing for the left_id

 if none of them, then traverse them with all 
 possible combinations
 • a_left - b_left
 • a_left - b_right
 • a_right - b_left
 • a_right - b_right
*/
oxb_noinline static void traverse_pair_recursive(oxb_bvh2d *const bvh, oxb_pool_id a_id, oxb_pool_id b_id, void (*func)(oxb_handle, oxb_handle, void *), void *ptr, uint32_t depth, oxb_bvh2d_traversalinfo *info) {
 if(oxb_pool_id_isnull(a_id) || oxb_pool_id_isnull(b_id))
  return;

 oxb_bvh2d_node *a = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, a_id);
 oxb_bvh2d_node *b = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, b_id);
 
 if(!overlaps(a->_aabb, b->_aabb))
  return;

 bool a_leaf = is_leaf(*a);
 bool b_leaf = is_leaf(*b);
 
 info->_node_visits++;
 info->_max_depth = info->_max_depth < depth ? depth : info->_max_depth;

 if(a_leaf && b_leaf) {
  if(oxb_pool_id_notequal(a_id, b_id)) {
   info->_pair_test++;
   func(a->_body, b->_body, ptr);
  }
  return;
 }
 if(a_leaf) {
  traverse_pair_recursive(bvh, a_id, b->_left, func, ptr, depth + 1, info);
  traverse_pair_recursive(bvh, a_id, b->_right, func, ptr, depth + 1, info);
  return;
 }
 if(b_leaf) {
  traverse_pair_recursive(bvh, a->_left, b_id, func, ptr, depth + 1, info);
  traverse_pair_recursive(bvh, a->_right, b_id, func, ptr, depth + 1, info);
  return;
 }
 traverse_pair_recursive(bvh, a->_left, b->_left, func, ptr, depth + 1, info);
 traverse_pair_recursive(bvh, a->_left, b->_right, func, ptr, depth + 1, info);
 traverse_pair_recursive(bvh, a->_right, b->_left, func, ptr, depth + 1, info);
 traverse_pair_recursive(bvh, a->_right, b->_right, func, ptr, depth + 1, info);
}


oxb_noinline static void traverse_self_recursive(oxb_bvh2d *const bvh, oxb_pool_id n_id, void (*func)(oxb_handle, oxb_handle, void *), void *ptr, uint32_t depth, oxb_bvh2d_traversalinfo *info) {
 if(oxb_pool_id_isnull(n_id))
  return;

 oxb_bvh2d_node *n = (oxb_bvh2d_node *)oxb_pool_get(&bvh->_pool, n_id);
 if(is_leaf(*n)) return;

 info->_node_visits++;
 info->_max_depth = info->_max_depth < depth ? depth : info->_max_depth;

 traverse_self_recursive(bvh, n->_left, func, ptr, depth + 1, info);
 traverse_self_recursive(bvh, n->_right, func, ptr, depth + 1, info);
 traverse_pair_recursive(bvh, n->_left, n->_right, func, ptr, depth + 1, info);
}

/* query collision for each body (thread safety) */
oxb_noinline static void traverse_single_body(oxb_bvh2d *const bvh, oxb_pool_id t, void (*func)(oxb_handle, oxb_handle, void *), void *ptr, const oxb_pool_id id, uint32_t depth, oxb_bvh2d_traversalinfo *info) {
 if(oxb_pool_id_isnull(t))
  return;

 if(oxb_pool_id_isequal(id, t))
  return;
  
 oxb_bvh2d_node *a = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, id);
 oxb_bvh2d_node *b = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, t);
 
 assert(is_leaf(*a));
 if(!overlaps(a->_aabb, b->_aabb))
  return;

 if(is_leaf(*b)) {
  info->_pair_test++;
  func(a->_body, b->_body, ptr);
  return;
 }

 info->_node_visits++;
 info->_max_depth = info->_max_depth < depth ? depth : info->_max_depth;

 traverse_single_body(bvh, b->_left, func, ptr, id, depth + 1, info);
 traverse_single_body(bvh, b->_right, func, ptr, id, depth + 1, info);
}

/* point query */
oxb_noinline static oxb_handle traverse_single_point(oxb_bvh2d *const bvh, oxb_pool_id t, oxb_pool2d *oxb_restrict const pool2d, oxb_vec2 p, uint32_t depth, oxb_bvh2d_traversalinfo *info) {
 if(oxb_pool_id_isnull(t))
  return OXB_HANDLE_NULL;
  
 oxb_bvh2d_node *a = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, t);
 
 if(a->_aabb._min._arr[0] > oxb_vec2_getx(p) || a->_aabb._min._arr[1] > oxb_vec2_gety(p)
  || a->_aabb._max._arr[0] < oxb_vec2_getx(p) || a->_aabb._max._arr[1] < oxb_vec2_gety(p))
  return OXB_HANDLE_NULL;

 if(is_leaf(*a)) {
  info->_pair_test++;
  oxb_collider2d collider;
  oxb_pool2d_collider(pool2d, &collider, a->_body);
  bool intersect = oxb_intersect2d_point_vs_object[collider._flags & OXB_SHAPE_MASK_2D](p, &collider);
  if(intersect)
   return a->_body;
  return OXB_HANDLE_NULL;
 }

 info->_node_visits++;
 info->_max_depth = info->_max_depth < depth ? depth : info->_max_depth;

 oxb_handle h = traverse_single_point(bvh, a->_left, pool2d, p, depth + 1, info);
 if(!oxb_handle_isequal(h, OXB_HANDLE_NULL))
  return h;
 return traverse_single_point(bvh, a->_right, pool2d, p, depth + 1, info);
}


oxb_noinline static void traverse_delete_non_leaf(oxb_bvh2d *const bvh, oxb_pool_id n) {
 if(oxb_pool_id_isnull(n)) {
   return;
 }
 oxb_bvh2d_node *node = (oxb_bvh2d_node *)oxb_pool_get(&bvh->_pool, n);

 if(is_leaf(*node))
  return;
 
 traverse_delete_non_leaf(bvh, node->_left);
 traverse_delete_non_leaf(bvh, node->_right);
 oxb_pool_release(&bvh->_pool, n);
}




/*
 Top down median split O(n log(n))
 not nearly as optimal, bit still balance and fast overall
*/


/*
 partition median
 re aranges the object data based on its position and opposite extent.
 
 for x-axis
 if the object is lower than the pivot (global center point)
 they will be reordered at the left side, otherwise they go tp the right side
 
 same for y-axis
 
 and then return the center index.
*/

oxb_noinline static int compare_leaf_x(const oxb_pool_id a, const oxb_pool_id b, oxb_bvh2d *const bvh) {
 oxb_bvh2d_node *node_a = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, a);
 oxb_bvh2d_node *node_b = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, b);
    
 float ca = node_a->_aabb._min._arr[0] + node_a->_aabb._max._arr[0];
 float cb = node_b->_aabb._min._arr[0] + node_b->_aabb._max._arr[0];
 return (int)(ca > cb) - (int)(ca < cb);
}

oxb_noinline static int compare_leaf_y(const oxb_pool_id a, const oxb_pool_id b, oxb_bvh2d *const bvh) {
 oxb_bvh2d_node *node_a = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, a);
 oxb_bvh2d_node *node_b = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, b);
    
 float ca = node_a->_aabb._min._arr[1] + node_a->_aabb._max._arr[1];
 float cb = node_b->_aabb._min._arr[1] + node_b->_aabb._max._arr[1];
 return (int)(ca > cb) - (int)(ca < cb);
}


oxb_noinline void quick_select_id_x(oxb_bvh2d *const bvh, oxb_pool_id *base, size_t left, size_t right, size_t target) {
 while(left < right) {
  size_t i = left;
  size_t j = right;

  oxb_pool_id pivot = base[left + ((right - left) >> 1)];
  while(i <= j) {
   while(compare_leaf_x(base[i], pivot, bvh) < 0)
    i++;
   while(compare_leaf_x(pivot, base[j], bvh) < 0)
    j--;
   if(i <= j) {
    {
     /* swap */
     oxb_pool_id tmp = base[i];
     base[i] = base[j];
     base[j] = tmp;
    }
    i++;
    if(j == left)
     break;
    j--;
   }
  }
  if(target <= j) {
   right = j;
  } else if (target >= i) {
   left = i;
  } else {
   break; /* between r and j */
  }
 }
}

oxb_noinline void quick_select_id_y(oxb_bvh2d *const bvh, oxb_pool_id *base, size_t left, size_t right, size_t target) {
 while(left < right) {
  size_t i = left;
  size_t j = right;

  oxb_pool_id pivot = base[left + ((right - left) >> 1)];
  while(i <= j) {
   while(compare_leaf_y(base[i], pivot, bvh) < 0)
    i++;
   while(compare_leaf_y(pivot, base[j], bvh) < 0)
    j--;
   if(i <= j) {
    {
     /* swap */
     oxb_pool_id tmp = base[i];
     base[i] = base[j];
     base[j] = tmp;
    }
    i++;
    if(j == left)
     break;
    j--;
   }
  }
  if(target <= j) {
   right = j;
  } else if (target >= i) {
   left = i;
  } else {
   break; /* between r and j */
  }
 }
}


/*
 spatial partition
 
 the larger extent will be choosen, lets say X
 it will be like this




 [Entire Region]
 
   Group A         Group B
 ________________________________
 |              |               |
 |              |               |
 |              |               |
 |              |               |
 |              |               |
 |              |               |
 |              |               |
 |______________|_______________|

 they will be devided into 2 groups of object
 allocate a new BVH node then it will become a parent of those 2

       •  --> new parent allocated
      / \
     /   \
    •     •  --> Group B (subtree)
    |
    |--------> Group A (subtree)


 then... recursively do the same thing again.






 [Right Side]
 _______________
 |              |
 |              | Group A
 |              |
 |______________|
 |              |
 |              | Group B
 |              |
 |______________|

       •
      / \
     /   \
    •     • --> allocate a new parent then repeat
         / \
        /   \
       •     • --> Group B (subtree)
       |
       |--------> Group A (subtree)
      
 because Y-extent is larger, it will be splitted into Y axis
 then recursive again






 [Top Side]

   GA       GB    --> group A and group B
  ______________
 |      |       |
 |      |       | Group B
 |      |       |
 |______|_______|

       •
      / \
     /   \
    •     •
         / \
        /   \
       •     •
      / \
     /   \
    •     • --> Group B(subtree)
    |
    |--------> Group A(subtree)
    
 repeats until they it encodes everything into the 
 BVH tree
 
*/
static oxb_pool_id build_top_down_recursive(oxb_bvh2d *const bvh, oxb_pool_id *leafs, size_t count) {
 if(count == 0) return OXB_POOL_NULL_ID;
 if(count == 1) return leafs[0];
 
 oxb_aabb2d centroid_bounds;
 
 oxb_vec2 centroid_min = oxb_vec2_splat(oxb_inf());
 oxb_vec2 centroid_max = oxb_vec2_splat(-oxb_inf());

 for(size_t i = 0; i < count; ++i) {
  oxb_bvh2d_node *leaf = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, leafs[i]);
  float center_x = (leaf->_aabb._min._arr[0] + leaf->_aabb._max._arr[0]) * 0.5f;
  float center_y = (leaf->_aabb._min._arr[1] + leaf->_aabb._max._arr[1]) * 0.5f;
  
  centroid_min = oxb_vec2_min(centroid_min, OXB_VEC2_INIT(center_x, center_y));
  centroid_max = oxb_vec2_max(centroid_max, OXB_VEC2_INIT(center_x, center_y));
 }

 oxb_vec2_store(centroid_bounds._min._arr, centroid_min);
 oxb_vec2_store(centroid_bounds._max._arr, centroid_max);

 oxb_vec2 extent = oxb_vec2_sub(centroid_max, centroid_min);
 float extent_x = oxb_vec2_getx(extent);
 float extent_y = oxb_vec2_gety(extent);
 
 int axis = (extent_x > extent_y) ? 0 : 1;
 size_t mid = 0;
 if(count > 1) {
  mid = count >> 1;
  if(axis == 0)
   quick_select_id_x(bvh, leafs, 0, count - 1, mid);
  else
   quick_select_id_y(bvh, leafs, 0, count - 1, mid);
 }

 /*
  then allocate a new BVH node.
  this would act as a parent of those 2 partition
 */
 oxb_pool_id parent_id = oxb_pool_acquire(&bvh->_pool);
 oxb_bvh2d_node *parent = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, parent_id);
 parent->_allocation = parent_id;
 parent->_parent = OXB_POOL_NULL_ID;
    
 oxb_pool_id left_id  = build_top_down_recursive(bvh, leafs, mid);
 oxb_pool_id right_id = build_top_down_recursive(bvh, leafs + mid, count - mid);

 parent = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, parent_id);
 parent->_left = left_id;
 parent->_right = right_id;

 oxb_bvh2d_node *left_node  = (oxb_bvh2d_node *)oxb_pool_get(&bvh->_pool, left_id);
 oxb_bvh2d_node *right_node = (oxb_bvh2d_node *)oxb_pool_get(&bvh->_pool, right_id);

 left_node->_parent = parent_id;
 right_node->_parent = parent_id;
 parent->_aabb = combine(left_node->_aabb, right_node->_aabb);
 return parent_id;
}






void oxb_bvh2d_create(oxb_bvh2d *oxb_restrict const bvh, oxb_allocator *oxb_restrict const alloc, const size_t pool_size, const size_t bump_size, const float aabb_margin) {
 bvh->_leaf_size = 0;
 bvh->_aabb_margin = aabb_margin;
 bvh->_root = OXB_POOL_NULL_ID;
 bvh->_frame_tolerance = 0;
 bvh->_checked = false;
 oxb_pool_create(&bvh->_pool, alloc, sizeof(oxb_bvh2d_node), pool_size);
 oxb_bump_create(&bvh->_bump, alloc, bump_size);
}


void oxb_bvh2d_destroy(oxb_bvh2d *oxb_restrict const bvh) {
 oxb_pool_destroy(&bvh->_pool);
 oxb_bump_destroy(&bvh->_bump);
}


void oxb_bvh2d_update(oxb_bvh2d *oxb_restrict const bvh, const oxb_pool_id id, const oxb_aabb2d fit) {
 oxb_bvh2d_node *leaf = (oxb_bvh2d_node*)oxb_pool_get(&bvh->_pool, id);
 if(contains(leaf->_aabb, fit))
  return;
 remove_leaf(bvh, id);
 leaf->_aabb = expanded(fit, bvh->_aabb_margin);
 leaf->_parent = OXB_POOL_NULL_ID;
 leaf->_left = OXB_POOL_NULL_ID;
 leaf->_right = OXB_POOL_NULL_ID;
 insert_leaf(bvh, id);
}


oxb_pool_id oxb_bvh2d_insert(oxb_bvh2d *oxb_restrict const bvh, const oxb_aabb2d fit, const oxb_handle body_id) {
 oxb_pool_id new_leaf = create_leaf(bvh, expanded(fit, bvh->_aabb_margin), body_id);
 insert_leaf(bvh, new_leaf);
 bvh->_leaf_size++;
 return new_leaf;
}


void oxb_bvh2d_remove(oxb_bvh2d *oxb_restrict const bvh, const oxb_pool_id id) {
 remove_leaf(bvh, id);
 oxb_pool_release(&bvh->_pool, id);
 bvh->_leaf_size--;
}



void oxb_bvh2d_traversepairs(oxb_bvh2d *oxb_restrict const bvh, void (*func)(oxb_handle, oxb_handle, void*), void *ptr) {
 oxb_bvh2d_traversalinfo tree_info;
 oxb_memory_set(&tree_info, 0, sizeof(oxb_bvh2d_traversalinfo));
 traverse_self_recursive(bvh, bvh->_root, func, ptr, 0, &tree_info);

 float tolerance_factor = 2.5f;
 float log_n = oxb_log2((float)bvh->_leaf_size);
 uint32_t max_depth = oxb_round(tolerance_factor * log_n);
 
 int ca = tree_info._max_depth > max_depth;
 int cb = tree_info._node_visits > (bvh->_leaf_size * 20u);
 int cc = tree_info._pair_test > (bvh->_leaf_size * 40u);

 if(ca && cb && cc)
  bvh->_frame_tolerance++;
 else
  bvh->_frame_tolerance = 0;
}


void oxb_bvh2d_traversebody(oxb_bvh2d *oxb_restrict const bvh, void (*func)(oxb_handle, oxb_handle, void*), void *ptr, const oxb_pool_id id) {
 oxb_bvh2d_traversalinfo tree_info;
 oxb_memory_set(&tree_info, 0, sizeof(oxb_bvh2d_traversalinfo));
 traverse_single_body(bvh, bvh->_root, func, ptr, id, 0, &tree_info);
 
 /* this makes it once per frame */
 if(!bvh->_checked) {
  float tolerance_factor = 2.5f;
  float log_n = oxb_log2((float)bvh->_leaf_size);
  uint32_t max_depth = oxb_round(tolerance_factor * log_n);
 
  int ca = tree_info._max_depth > max_depth;
  int cb = tree_info._node_visits > (bvh->_leaf_size * 20u);
  int cc = tree_info._pair_test > (bvh->_leaf_size * 40u);

  if(ca && cb && cc)
   bvh->_frame_tolerance++;
  else
   bvh->_frame_tolerance = 0;
  bvh->_checked = true;
 }
}


void oxb_bvh2d_mediansplit(oxb_bvh2d *oxb_restrict const bvh, oxb_pool2d *oxb_restrict pool2d) {
 traverse_delete_non_leaf(bvh, bvh->_root);
 bvh->_root = OXB_POOL_NULL_ID;
 
 /* TODO : add sensor */
 oxb_dynamic2d *dynamics = (oxb_dynamic2d *)pool2d->_containers[OXB_POOL2D_TYPE_DYNAMIC_2D]._bytes;
 const size_t dynamics_size = (size_t)pool2d->_containers[OXB_POOL2D_TYPE_DYNAMIC_2D]._size;

 oxb_kinematic2d *kinematics = (oxb_kinematic2d*)pool2d->_containers[OXB_POOL2D_TYPE_KINEMATIC_2D]._bytes;
 const size_t kinematics_size = (size_t)pool2d->_containers[OXB_POOL2D_TYPE_KINEMATIC_2D]._size;

 oxb_static2d *statics = (oxb_static2d*)pool2d->_containers[OXB_POOL2D_TYPE_STATIC_2D]._bytes;
 const size_t statics_size = (size_t)pool2d->_containers[OXB_POOL2D_TYPE_STATIC_2D]._size;


 size_t total_size = dynamics_size + kinematics_size + statics_size;
 assert(total_size == bvh->_leaf_size);
 if(total_size == 0)
  return;
 
 size_t offset = oxb_bump_acquire(&bvh->_bump, sizeof(oxb_pool_id) * total_size, bvh->_pool._allocator->_alignment);
 oxb_pool_id *leafs = oxb_bump_get(&bvh->_bump, offset);
 
 size_t current_offset = 0;
 for(size_t i = 0; i < dynamics_size; i++) {
  leafs[current_offset + i] = dynamics[i]._bvh_id;
 }
 current_offset += dynamics_size;
 for(size_t i = 0; i < kinematics_size; i++) {
  leafs[current_offset + i] = kinematics[i]._bvh_id;
 }
 current_offset += kinematics_size;
 for(size_t i = 0; i < statics_size; i++) {
  leafs[current_offset + i] = statics[i]._bvh_id;
 }
 
 oxb_compiler_memory_barrier();
 
 bvh->_root = build_top_down_recursive(bvh, leafs, total_size);
 oxb_bump_point(&bvh->_bump, 0);
}



bool oxb_bvh2d_shouldrebuild(oxb_bvh2d *oxb_restrict const bvh, const uint8_t frame_tolerance) {
 bvh->_checked = false;
 return bvh->_frame_tolerance >= frame_tolerance;
}

oxb_handle oxb_bvh2d_querypoint(oxb_bvh2d *oxb_restrict const bvh, oxb_pool2d *oxb_restrict const pool2d, oxb_vec2 point) {
 oxb_bvh2d_traversalinfo tree_info;
 oxb_memory_set(&tree_info, 0, sizeof(oxb_bvh2d_traversalinfo));
 oxb_handle h = traverse_single_point(bvh, bvh->_root, pool2d, point, 0, &tree_info);
 
 /* this makes it once per frame */
 if(!bvh->_checked) {
  float tolerance_factor = 2.5f;
  float log_n = oxb_log2((float)bvh->_leaf_size);
  uint32_t max_depth = oxb_round(tolerance_factor * log_n);
 
  int ca = tree_info._max_depth > max_depth;
  int cb = tree_info._node_visits > (bvh->_leaf_size * 20u);
  int cc = tree_info._pair_test > (bvh->_leaf_size * 40u);

  if(ca && cb && cc)
   bvh->_frame_tolerance++;
  else
   bvh->_frame_tolerance = 0;
  bvh->_checked = true;
 }
 return h;
}

