enum Dir { NORTH=0, EAST, SOUTH, WEST };


int main() {
    // 1) initialize
    Junction* root = new Junction(nullptr, NORTH);
    scanExits(root);
    Junction* curr = root;

    // 2) DFS loop
    while (curr) {
        Dir nextDir;
        if (popExit(curr, nextDir))// {
            // explore that branch
            turnTo(nextDir); 
            driveToNextJunction();

            Junction* child = new Junction(curr, opposite(nextDir));
            scanExits(child);
            curr = child;
        }
        else {
            // backtrack
            if (!curr->parent) break;
            turnTo(curr->entryDir);
            driveToNextJunction();
            curr = curr->parent;
        }
    }

    std::cout << "Exploration complete!\n";
    return 0;
}