using Pkg
packages = keys(Pkg.installed())
if !in("AutomotiveSimulator", packages)
    Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotiveSimulator.jl.git"))
end
if !in("AutomotiveVisualization", packages)
    Pkg.add(PackageSpec(url="https://github.com/sisl/AutomotiveVisualization.jl.git"))
end
if !in("EzXML", packages)
    Pkg.add(PackageSpec(url="https://github.com/bicycle1885/EzXML.jl"))
end