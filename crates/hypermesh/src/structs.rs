
enum CsgNodeType { Union, Intersection, Difference, Leaf }

trait CsgNode {
    fn ToLeafNode () {

    }
}

struct CsgOpNode {
    
}


struct CsgLeafNode {
    
}

impl CsgNode for CsgOpNode {
    fn ToLeafNode () {
        
    }
}

impl CsgNode for CsgLeafNode {

}
