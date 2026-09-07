
using System.Text.Json;
using System.Text.Json.Serialization;

namespace LivoxHmi.Core;

public sealed class ProjectStore
{
    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        WriteIndented = true,
        PropertyNameCaseInsensitive = true,
        Converters = { new JsonStringEnumConverter() }
    };

    public async Task SaveAsync(ProjectDefinition project, string path, CancellationToken ct = default)
    {
        var directory = Path.GetDirectoryName(Path.GetFullPath(path))!;
        Directory.CreateDirectory(directory);

        var temp = path + ".tmp";
        var backup = path + ".bak";

        await using (var fs = new FileStream(temp, FileMode.Create, FileAccess.Write, FileShare.None,
                                             64 * 1024, FileOptions.WriteThrough | FileOptions.Asynchronous))
        {
            await JsonSerializer.SerializeAsync(fs, project, JsonOptions, ct);
            await fs.FlushAsync(ct);
        }

        if (File.Exists(path))
        {
            File.Copy(path, backup, true);
        }

        File.Move(temp, path, true);
    }

    public async Task<ProjectDefinition> LoadAsync(string path, CancellationToken ct = default)
    {
        await using var fs = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read,
                                            64 * 1024, FileOptions.Asynchronous);
        return await JsonSerializer.DeserializeAsync<ProjectDefinition>(fs, JsonOptions, ct)
               ?? throw new InvalidDataException("Project file is empty or invalid.");
    }
}
