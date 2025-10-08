#include "RockGazeboHelpers.hpp"
#include <gazebo/common/Exception.hh>
#include <regex>

using namespace std;

sdf::ElementPtr rock_gazebo_helpers::findPluginElement(sdf::ElementPtr enclosing,
    string const& file_name)
{
    sdf::ElementPtr plugin_element = enclosing->GetElement("plugin");
    while (plugin_element) {
        if (plugin_element->Get<string>("filename") == file_name) {
            gzmsg << "Found plugin: " << plugin_element->Get<string>("name") << " ("
                  << file_name << ")" << endl;
            return plugin_element;
        }
        plugin_element = plugin_element->GetNextElement("plugin");
    }
    return sdf::ElementPtr();
}

sdf::ElementPtr rock_gazebo_helpers::findPluginElementByName(sdf::ElementPtr enclosing,
    string const& plugin_name)
{
    sdf::ElementPtr plugin_element = enclosing->GetElement("plugin");
    while (plugin_element) {
        if (plugin_element->Get<string>("name") == plugin_name) {
            gzmsg << "Found plugin: " << plugin_element->Get<string>("name") << " ("
                  << plugin_name << ")" << endl;
            return plugin_element;
        }
        plugin_element = plugin_element->GetNextElement("plugin");
    }
    return sdf::ElementPtr();
}

sdf::ElementPtr rock_gazebo_helpers::getPluginElement(sdf::ElementPtr enclosing,
    string const& file_name)
{
    auto element = findPluginElement(enclosing, file_name);

    if (!element) {
        // TODO: change this error message to be more generic
        string msg = "GazeboThruster: sdf model loaded the thruster plugin, but it\n"
                     "cannot be found in the SDF object. Expected the thruster plugin\n"
                     "filename to be libgazebo_thruster.so\n";
        gzthrow(msg);
    }

    return element;
}

sdf::ElementPtr rock_gazebo_helpers::getPluginElementByName(sdf::ElementPtr enclosing,
    string const& plugin_name)
{
    auto element = findPluginElementByName(enclosing, plugin_name);

    if (!element) {
        string msg =
            "Unable to find any plugin named " + plugin_name + " in the SDF object.\n";
        gzthrow(msg);
    }

    return element;
}

string rock_gazebo_helpers::computeModelTopicScope(gazebo::physics::ModelPtr model)
{
    return "/gazebo/" + regex_replace(model->GetScopedName(true), regex("::"), "/");
}

string rock_gazebo_helpers::computePluginTopicScope(gazebo::physics::ModelPtr model,
    sdf::ElementPtr plugin)
{
    string plugin_name = plugin->Get<string>("name");
    if (plugin_name.find("__") != string::npos) {
        return "/" + regex_replace(plugin_name, regex("__"), "/");
    }
    else {
        string full_gazebo_name =
            "gazebo::" + model->GetScopedName(true) + "::" + plugin_name;
        return "/" + regex_replace(full_gazebo_name, regex("::"), "/");
    }
}

static string resolveLinkScopeFromDoubleUndescoredPluginName(
    gazebo::physics::ModelPtr model,
    string const& plugin_name)
{
    // Do slightly better than the actual old implementation, and validate that
    // the beginning of the full path is the fully scoped model name
    auto expected_prefix_gazebo = "gazebo::" + model->GetScopedName(true);
    auto expected_prefix = regex_replace(expected_prefix_gazebo, regex("::"), "__");
    if (plugin_name.substr(0, expected_prefix.size()) != expected_prefix) {
        gzthrow("expected " + plugin_name + " to start with " + expected_prefix);
    }

    auto rfind = plugin_name.rfind("__");
    if (rfind == expected_prefix.size()) {
        return "";
    }

    auto relative_scope = plugin_name.substr(expected_prefix.size() + 2,
        rfind - expected_prefix.size() - 2);

    cout << "name: " << plugin_name << "\n";
    cout << "expected: " << expected_prefix << "\n";
    cout << "rfind: " << rfind << "\n";
    cout << "relative_scope: " << relative_scope << "\n";

    return regex_replace(relative_scope, regex("__"), "::");
}

static string resolvePluginParentScope(gazebo::physics::ModelPtr model,
    string const& plugin_name)
{
    if (plugin_name.find("__") != string::npos) {
        return resolveLinkScopeFromDoubleUndescoredPluginName(model, plugin_name);
    }
    else {
        return plugin_name.substr(0, plugin_name.rfind("::"));
    }
}

static string applyScope(string const& scope, string const& name)
{
    if (scope.empty()) {
        return name;
    }
    else {
        return scope + "::" + name;
    }
}

gazebo::physics::LinkPtr rock_gazebo_helpers::resolveLink(gazebo::physics::ModelPtr model,
    sdf::ElementPtr plugin,
    string const& link_name)
{
    string plugin_name = plugin->Get<string>("name");

    auto scope = resolvePluginParentScope(model, plugin_name);
    auto full_link_name = applyScope(scope, link_name);

    auto link = model->GetLink(full_link_name);
    if (!link) {
        string msg = "could not find link " + link_name + " specified in plugin " +
                     plugin_name + " (full link name resolved to " + full_link_name + ")";
        gzthrow(msg);
    }
    return link;
}

gazebo::physics::LinkPtr rock_gazebo_helpers::resolveLinkWithDefault(
    gazebo::physics::ModelPtr model,
    sdf::ElementPtr plugin_sdf,
    string const& element_name)
{
    if (plugin_sdf->HasElement(element_name)) {
        auto link_name = plugin_sdf->Get<string>(element_name);
        return resolveLink(model, plugin_sdf, link_name);
    }
    else if (model->GetLinks().empty()) {
        gzthrow("No link defined in reference model, and no " + element_name +
                " element found plugin");
    }
    else {
        auto plugin_name = plugin_sdf->Get<string>("name");
        auto scope = resolvePluginParentScope(model, plugin_name);
        auto it = find_if(model->GetLinks().begin(),
            model->GetLinks().end(),
            [&](auto link_ptr) -> bool {
                auto s = link_ptr->GetName();
                return s.substr(0, scope.size()) == scope;
            });

        if (it == model->GetLinks().end()) {
            gzthrow("Element " + element_name +
                    " missing in plugin definition, attempted "
                    "to find a link whose scoped name starts with " +
                    scope +
                    " but found "
                    "none");
        }

        gzmsg << "Element " << element_name
              << " missing in plugin definition, picking first "
                 "link of the enclosing model, "
              << (*it)->GetScopedName() << endl;
        return *it;
    }
}
