/**
 * Custom Navbar Item Component Types
 *
 * Extends default Docusaurus navbar items with custom types
 * - Adds support for 'custom-langSwitcher' type
 * - Adds support for 'custom-authNavbarItem' type
 * - Adds support for 'custom-search' type
 */
import ComponentTypes from "@theme-original/NavbarItem/ComponentTypes";
import AuthNavbarItem from "@site/src/components/AuthNavbarItem";
import SearchNavbarItem from "@site/src/components/SearchNavbarItem";
import LangSwitcher from "@site/src/components/LangSwitcher";

export default {
  ...ComponentTypes,
  "custom-authNavbarItem": AuthNavbarItem,
  "custom-search": SearchNavbarItem,
  "custom-langSwitcher": LangSwitcher,
};
