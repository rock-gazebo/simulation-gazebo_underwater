#ifndef GAZEBO_THRUSTER_UTILITIES_HPP
#define GAZEBO_THRUSTER_UTILITIES_HPP

#include <gazebo/common/Console.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

/** Generic helpers helpful to develop plugins in relation (but with no dependencies on)
 * rock-gazebo
 *
 * Using these heleprs do NOT create a dependency on rock-gazebo. They are generic
 * helpers for the most part, and also implement the way rock-gazebo allows to
 * handle hierarchical models meaningfully. This last part is a set of
 * principles/best practices and not a direct dependency.
 *
 * We did not want to create a full package to distribute them ... they are meant
 * to be copied to the package(s) where they are needed.
 */
namespace rock_gazebo_helpers {
    /** Looks for a <plugin> element
     *
     * This methods searches for a plugin element, direct child of
     * \c enclosing, whose filename is the given file name. It differs
     * from \c getPluginElement by the way it handles the missing case.
     *
     * @return a null pointer if the plugin could not be found
     * @see getPluginElement
     */
    sdf::ElementPtr findPluginElement(sdf::ElementPtr enclosing,
        std::string const& fileName);

    /** Looks for a <plugin> element
     *
     * This methods searches for a plugin element, direct child of
     * \c enclosing, whose name is the given name. It differs
     * from \c getPluginElement by the way it handles the missing case.
     *
     * @return a null pointer if the plugin could not be found
     * @see getPluginElement
     */
    sdf::ElementPtr findPluginElementByName(sdf::ElementPtr enclosing,
        std::string const& plugin_name);

    /** Resolves for a <plugin> element
     *
     * This methods searches for a plugin element, direct child of
     * \c enclosing, whose filename is the given file name. It differs
     * from \c findPluginElement by the way it handles the missing case.
     *
     * @throw invalid_argument if the plugin could not be found
     * @see findPluginElement
     */
    sdf::ElementPtr getPluginElement(sdf::ElementPtr enclosing,
        std::string const& fileName);

    /** Resolves for a <plugin> element
     *
     * This methods searches for a plugin element, direct child of
     * \c enclosing, whose name is the given name. It differs
     * from \c findPluginElementByName by the way it handles the missing case.
     *
     * @throw invalid_argument if the plugin could not be found
     * @see findPluginElement
     */
    sdf::ElementPtr getPluginElementByName(sdf::ElementPtr enclosing,
        std::string const& plugin_name);

    /** Get a typed parameter from the given element
     *
     * If the parameter does not exist, use a default value
     */
    template <class T>
    T getParameter(std::string plugin_name,
        sdf::ElementPtr element,
        std::string parameter_name,
        std::string dimension,
        T default_value)
    {
        gzmsg << plugin_name << ": " << parameter_name;
        if (element->HasElement(parameter_name.c_str())) {
            T var = element->Get<T>(parameter_name.c_str());
            gzmsg << "=" << var << " " << dimension << std::endl;
            return var;
        }
        else {
            gzmsg << " using default " << default_value << " " << dimension << std::endl;
            return default_value;
        }
    }

    /** Get the scope of a topic that is relative to the given model
     */
    std::string computeModelTopicScope(gazebo::physics::ModelPtr model);

    /** Get the scope of a topic that is published by the given plugin
     */
    std::string computePluginTopicScope(gazebo::physics::ModelPtr model,
        sdf::ElementPtr plugin);

    /** Compute the actual name of a link based on a relative link name from the SDF
     *
     * To account for model inclusions, a plugin in an included model that has to
     * resolve a link name will have to prepend the path between the root model
     * and the parent model of the plugin.
     *
     * This method assumes that the plugin name has been re-scoped that way, i.e.
     * that the relative path to the root is simply everything before :: in
     * the plugin name
     */
    gazebo::physics::LinkPtr resolveLink(gazebo::physics::ModelPtr model,
        sdf::ElementPtr plugin,
        std::string const& link_name);

    /** Resolve a link from name if an attribute provides it, or return the first link
     * of the model
     *
     * @param model the reference gazebo model. If no link is defined in the plugin
     *    description, its first link is returned instead.
     * @param plugin the plugin SDF definition
     * @param element_name the name of the element that contains the link name. If
     *    it not present, the function will return the first link of the model.
     */
    gazebo::physics::LinkPtr resolveLinkWithDefault(gazebo::physics::ModelPtr model,
        sdf::ElementPtr plugin_sdf,
        std::string const& element_name);
}

#endif