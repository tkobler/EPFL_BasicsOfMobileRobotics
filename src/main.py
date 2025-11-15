from tdmclient import ClientAsync, aw # Warning: for the report, imports must be clearly stated at the top of the file
import filter
import global_nav
import local_nav
import motion_control
import vision

async def main():
    # remark: see doc to know how to use this method: https://github.com/epfl-mobots/tdm-python/blob/main/doc/lowlevel.md
    try: 
        node = await client.wait_for_node()
        await node.lock()

        running = True
        while running:
            
            # Dummy example, it could be better to have classes for better separation of modules, variables and methods.
            # Here we just call each basic module's processing function (example function)
            # It will also probably be better to have a higher level control structure to manage the flow between modules ,
            # to hande the node's state, etc. (i.e. another class or something)
            
            filter.filter_processing()
            global_nav.global_nav_processing()
            local_nav.local_nav_processing()
            motion_control.motion_control_processing()
            vision.vision_processing()
        
            running = False  # Just run once for this example

        # Stop the program
        await node.stop()
        await node.unlock()

    except Exception as e:
        print(f"An error occurred: {e}")

    # Exit program
    exit(0)

if __name__ == "__main__": # runs the main function if this file is executed directly
    with ClientAsync() as client:
        client.run_async_program(main)